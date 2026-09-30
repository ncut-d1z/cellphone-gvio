"""Index guard regression tests; synthetic marker only, never real key material."""
from pathlib import Path
import subprocess
import sys
import tempfile

SCANNER = Path(__file__).with_name('check_no_private_keys.py').resolve()


def main() -> None:
    with tempfile.TemporaryDirectory(prefix='gvio-guard-') as directory:
        root = Path(directory)
        subprocess.run(['git', 'init', '--quiet', str(root)], check=True)

        def git(*args: str) -> None:
            subprocess.run(['git', '-C', str(root), *args], check=True, capture_output=True)

        def scan(expected: int) -> None:
            result = subprocess.run([sys.executable, str(SCANNER)], cwd=root, capture_output=True, text=True)
            assert result.returncode == expected, result.stdout + result.stderr

        (root / 'safe.txt').write_text('ordinary file', encoding='utf-8')
        git('add', 'safe.txt')
        scan(0)
        (root / 'SERVER.KEY').write_text('not even a real key', encoding='utf-8')
        git('add', 'SERVER.KEY')
        scan(1)
        git('rm', '--cached', 'SERVER.KEY')
        # Split the marker so this test source is not itself a private-key blob.
        marker = '-' * 5 + 'BEGIN ' + 'PRIVATE KEY' + '-' * 5
        (root / 'renamed.txt').write_text(marker + '\nsynthetic\n', encoding='utf-8')
        git('add', 'renamed.txt')
        (root / 'renamed.txt').write_text('working tree is clean but index is not', encoding='utf-8')
        scan(1)
        git('rm', '--cached', '-f', 'renamed.txt')
        scan(0)
        print('PASS key-name, renamed-key, staged-vs-working-tree and clean-index cases')


if __name__ == '__main__':
    main()
