#Requires -Version 5.1
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path $PSScriptRoot -Parent
$generator = Join-Path $root 'generate_private_key.ps1'
$certDir = Join-Path $root 'backend/certs'
$key = Join-Path $certDir 'server.key'
$cert = Join-Path $certDir 'server.crt'
$openssl = (Get-Command openssl -ErrorAction SilentlyContinue)
if ($openssl) { $exe = $openssl.Source } else {
    $git = (Get-Command git).Source
    $exe = Join-Path (Split-Path (Split-Path $git -Parent) -Parent) 'usr/bin/openssl.exe'
}
& $generator -OpenSSLPath $exe -KeyBits 2048 -Days 1
$before = (Get-FileHash -LiteralPath $key -Algorithm SHA256).Hash
$refused = $false
try { & $generator -OpenSSLPath $exe -KeyBits 2048 -Days 1 } catch { $refused = $true }
if (-not $refused) { throw 'Existing key was overwritten without -Force.' }
if ((Get-FileHash -LiteralPath $key -Algorithm SHA256).Hash -ne $before) { throw 'Refused generation changed the key.' }
& $generator -OpenSSLPath $exe -KeyBits 2048 -Days 1 -Force
if ((Get-FileHash -LiteralPath $key -Algorithm SHA256).Hash -eq $before) { throw 'Rotation did not change the key.' }
& $exe x509 -in $cert -noout -checkhost localhost
if ($LASTEXITCODE -ne 0) { throw 'Missing localhost SAN.' }
& $exe x509 -in $cert -noout -checkip 127.0.0.1
if ($LASTEXITCODE -ne 0) { throw 'Missing loopback SAN.' }
$publicFromKey = (& $exe pkey -in $key -pubout) -join "`n"
if ($LASTEXITCODE -ne 0) { throw 'Cannot read public key.' }
$publicFromCert = (& $exe x509 -in $cert -pubkey -noout) -join "`n"
if ($LASTEXITCODE -ne 0 -or $publicFromCert -ne $publicFromKey) { throw 'Key and certificate do not match.' }
if (-not (Get-Acl -LiteralPath $key).AreAccessRulesProtected) { throw 'Key ACL still inherits directory access.' }
$refused = $false
try { & $generator -OpenSSLPath $exe -OutputDirectory (Join-Path $root 'android-app/app/src/main/assets/certs') } catch { $refused = $true }
if (-not $refused) { throw 'Generator accepted an Android assets destination.' }
Push-Location $root
try {
    git check-ignore --quiet backend/certs/server.key
    if ($LASTEXITCODE -ne 0) { throw 'Generated key is not ignored.' }
} finally { Pop-Location }
Write-Host 'PASS generation, SAN, matching pair, ACL, overwrite refusal, rotation and APK-assets refusal'
