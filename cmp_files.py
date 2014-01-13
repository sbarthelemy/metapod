import sys
import filecmp
from os.path import join

def cmp_files(root0, root1, files):
    """
    Compare compares the files in the two directories root0 and root1,
    returning True if they seem equal, False otherwise.
    """
    for f in files:
        try:
            # do a shallow comparaison
            if not filecmp.cmp(join(root0, f),
                               join(root1, f)):
                print("error: file \"{0}\" differ in {1} {2}".format(f, root0, root1))
                return False
        except OSError:
            print("error: file \"{0}\" differ in {1} {2}".format(f, root0, root1))
            return False
    return True


if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python cmp_files.py root0 root1 [file0 file1 ...]")
        sys.exit(1)

    if not cmp_files(sys.argv[1], sys.argv[2], sys.argv[3:]):
        sys.exit(1)
