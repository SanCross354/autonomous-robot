
import sys
import glob

# Try importing pypdf or PyPDF2
try:
    from pypdf import PdfReader
except ImportError:
    try:
        from PyPDF2 import PdfReader
    except ImportError:
        print("ERROR: No pypdf or PyPDF2 found.")
        sys.exit(1)

files = glob.glob("*.pdf")
target = "IET Image Processing - 2021 - Tsai - A robust tracking algorithm for a human‐following mobile robot.pdf"

if target not in files:
    print(f"ERROR: File '{target}' not found.")
    sys.exit(1)

try:
    reader = PdfReader(target)
    text = ""
    for i, page in enumerate(reader.pages):
        text += f"--- Page {i+1} ---\n"
        text += page.extract_text() + "\n"
    
    with open("paper_text.txt", "w") as f:
        f.write(text)
    print("SUCCESS: usage extracted to paper_text.txt")

except Exception as e:
    print(f"ERROR: {e}")
