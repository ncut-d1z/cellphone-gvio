#Requires -Version 5.1
<#
.SYNOPSIS
Generate a LOCAL development TLS private key and matching self-signed certificate.
.EXAMPLE
.\generate_private_key.ps1
.EXAMPLE
.\generate_private_key.ps1 -Force -DnsNames localhost,my-pc -IpAddresses 127.0.0.1,192.168.1.10
.NOTES
Requires OpenSSL (PATH, Git for Windows, or -OpenSSLPath). Never copies a private
key into Android assets. Existing output is not overwritten without -Force.
The formerly committed key is compromised; deleting it does not purge history.
#>
[CmdletBinding()]
param(
    [string]$OutputDirectory = (Join-Path $PSScriptRoot 'backend/certs'),
    [string]$OpenSSLPath = '',
    [string[]]$DnsNames = @('localhost'),
    [string[]]$IpAddresses = @('127.0.0.1', '::1'),
    [ValidateSet(2048, 3072, 4096)][int]$KeyBits = 3072,
    [ValidateRange(1, 825)][int]$Days = 365,
    [switch]$Force
)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$isWindowsHost = $env:OS -eq 'Windows_NT'

function Protect-PrivatePath([string]$Path, [bool]$Directory) {
    if ($isWindowsHost) {
        $sid = [System.Security.Principal.WindowsIdentity]::GetCurrent().User
        if ($Directory) {
            $acl = New-Object System.Security.AccessControl.DirectorySecurity
            $rule = New-Object System.Security.AccessControl.FileSystemAccessRule(
                $sid, 'FullControl', 'ContainerInherit,ObjectInherit', 'None', 'Allow')
        } else {
            $acl = New-Object System.Security.AccessControl.FileSecurity
            $rule = New-Object System.Security.AccessControl.FileSystemAccessRule($sid, 'FullControl', 'Allow')
        }
        $acl.SetOwner($sid)
        $acl.SetAccessRuleProtection($true, $false)
        $acl.AddAccessRule($rule)
        Set-Acl -LiteralPath $Path -AclObject $acl
    } else {
        $mode = if ($Directory) { '700' } else { '600' }
        & chmod $mode -- $Path
        if ($LASTEXITCODE -ne 0) { throw "Could not restrict permissions: $Path" }
    }
}

if (-not $OpenSSLPath) {
    $cmd = Get-Command openssl -ErrorAction SilentlyContinue
    if ($cmd) { $OpenSSLPath = $cmd.Source }
    if (-not $OpenSSLPath) {
        $git = Get-Command git -ErrorAction SilentlyContinue
        if ($git) {
            $candidate = Join-Path (Split-Path (Split-Path $git.Source -Parent) -Parent) 'usr/bin/openssl.exe'
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { $OpenSSLPath = $candidate }
        }
    }
}
if (-not $OpenSSLPath -or -not (Test-Path -LiteralPath $OpenSSLPath -PathType Leaf)) {
    throw 'OpenSSL not found. Install OpenSSL or pass -OpenSSLPath with the executable path.'
}
if (($DnsNames.Count + $IpAddresses.Count) -eq 0) { throw 'At least one DNS name or IP address is required.' }
foreach ($name in $DnsNames) {
    if ($name -notmatch '^[A-Za-z0-9][A-Za-z0-9.-]*$') { throw "Invalid DNS name: $name" }
}
foreach ($ip in $IpAddresses) {
    $parsed = $null
    if (-not [System.Net.IPAddress]::TryParse($ip, [ref]$parsed)) { throw "Invalid IP address: $ip" }
}
$output = [System.IO.Path]::GetFullPath($OutputDirectory)
$assets = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'android-app/app/src/main/assets'))
$sep = [System.IO.Path]::DirectorySeparatorChar
if ($output.Equals($assets, [System.StringComparison]::OrdinalIgnoreCase) -or
    $output.StartsWith($assets + $sep, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Private keys must not be generated inside Android assets.'
}
New-Item -ItemType Directory -Path $output -Force | Out-Null
$key = Join-Path $output 'server.key'
$cert = Join-Path $output 'server.crt'
if (-not $Force -and ((Test-Path -LiteralPath $key) -or (Test-Path -LiteralPath $cert))) {
    throw 'Output already exists. Use -Force only when intentionally rotating the key/certificate pair.'
}
$temp = Join-Path $output ('.tls-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temp | Out-Null
$installedKey = $false
$installedCert = $false
$success = $false
try {
    Protect-PrivatePath $temp $true
    $san = @()
    $i = 0
    foreach ($name in $DnsNames) { $i++; $san += "DNS.$i = $name" }
    $i = 0
    foreach ($ip in $IpAddresses) { $i++; $san += "IP.$i = $ip" }
    $config = @(
        '[req]', 'prompt = no', 'distinguished_name = dn', 'x509_extensions = server_ext',
        '[dn]', 'CN = gvio-development', '[server_ext]', 'basicConstraints = critical,CA:FALSE',
        'keyUsage = critical,digitalSignature,keyEncipherment', 'extendedKeyUsage = serverAuth',
        'subjectAltName = @names', '[names]'
    ) + $san
    $configPath = Join-Path $temp 'openssl.cnf'
    [System.IO.File]::WriteAllLines($configPath, $config, [System.Text.Encoding]::ASCII)
    $newKey = Join-Path $temp 'server.key'
    $newCert = Join-Path $temp 'server.crt'
    & $OpenSSLPath req -x509 -newkey "rsa:$KeyBits" -sha256 -nodes -days $Days -config $configPath -keyout $newKey -out $newCert
    if ($LASTEXITCODE -ne 0) { throw 'OpenSSL key/certificate generation failed.' }
    Protect-PrivatePath $newKey $false
    & $OpenSSLPath pkey -in $newKey -check -noout
    if ($LASTEXITCODE -ne 0) { throw 'Private key validation failed.' }
    & $OpenSSLPath x509 -in $newCert -noout -subject -dates -fingerprint -sha256
    if ($LASTEXITCODE -ne 0) { throw 'Certificate validation failed.' }
    # Generate and validate the entire pair before replacing either existing file.
    if (Test-Path -LiteralPath $key) { Move-Item -LiteralPath $key -Destination (Join-Path $temp 'old.key') }
    if (Test-Path -LiteralPath $cert) { Move-Item -LiteralPath $cert -Destination (Join-Path $temp 'old.crt') }
    Move-Item -LiteralPath $newKey -Destination $key
    $installedKey = $true
    Protect-PrivatePath $key $false
    Move-Item -LiteralPath $newCert -Destination $cert
    $installedCert = $true
    $success = $true
} finally {
    if (-not $success) {
        if ($installedKey) { Remove-Item -LiteralPath $key -Force }
        if ($installedCert) { Remove-Item -LiteralPath $cert -Force }
        if (Test-Path -LiteralPath (Join-Path $temp 'old.key')) {
            Move-Item -LiteralPath (Join-Path $temp 'old.key') -Destination $key
        }
        if (Test-Path -LiteralPath (Join-Path $temp 'old.crt')) {
            Move-Item -LiteralPath (Join-Path $temp 'old.crt') -Destination $cert
        }
    }
    Remove-Item -LiteralPath $temp -Recurse -Force
}
Write-Host "Private key (local only): $key"
Write-Host "Public certificate: $cert"
Write-Warning 'Development certificate only. Do not commit/share the key or bundle it in an APK. Remove trust in the old certificate on every client.'
