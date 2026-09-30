# Local development TLS material

Run `./generate_private_key.ps1` from the repository root in PowerShell 5.1+.
OpenSSL must be installed; `-OpenSSLPath` accepts an explicit executable path.
The script generates an RSA private key and a matching self-signed certificate
with localhost DNS/IP subject alternative names. Add the actual host using
`-DnsNames` / `-IpAddresses`. Use `-Force` only for intentional rotation.

`server.key` and `server.crt` are generated locally and ignored by Git. The key is
unencrypted for unattended server startup; its filesystem permissions are
restricted to the current user. Never copy the key to Android assets, a public
web root, a build artifact, or another machine. A client may be provisioned with
only the public certificate after independently checking its fingerprint.

The pair committed before the security cleanup must not be reused. The old key
still exists in Git history and potentially in old APKs/clones. Removing files
from the current branch does not revoke an existing trust relationship or purge
history. Replace the pair and remove trust in the old certificate on clients.
No history rewrite is performed by this change.

`.gitignore` prevents ordinary accidental staging; `git add -f` can bypass it.
Run `python tools/check_no_private_keys.py` before committing. The same index
content check runs in CI. Enforce that CI check in branch protection separately
when protection is desired.
