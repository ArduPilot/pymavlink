import hashlib
import sys

def hash_sha256(s) -> bytes:
    if isinstance(s, str):
        s = s.encode("utf-8")
    h = hashlib.new('sha256')
    h.update(s)
    return h.digest()


def test_hash_sha256() -> None:
    test_cases = [
        ("", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"),
        ("hello", "2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824"),
        ("world", "486ea46224d1bb4fb680f34f7c9ad96a8f24ec88be73ea8e5a6c65260e9cb8a7"),
        (b"world", "486ea46224d1bb4fb680f34f7c9ad96a8f24ec88be73ea8e5a6c65260e9cb8a7"),
    ]

    for input_str, expected_hash in test_cases:
        assert hash_sha256(input_str).hex() == expected_hash, f"Failed for input: {input_str}"


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python sha256_test.py <string>")
        sys.exit(1)

    res = hash_sha256(sys.argv[1])
    for i in range(6):
        sys.stdout.write("%02x " % res[i])
    sys.stdout.write("\n")

if __name__ == "__main__":
    main()
