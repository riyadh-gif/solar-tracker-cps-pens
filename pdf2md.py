"""PDF -> Markdown dengan Docling.
Pakai: python pdf2md.py FOLDER [--skip POLA ...]   (hasil di FOLDER/md/)
Contoh: python pdf2md.py jurnalrandom --skip "978-*"
"""
import fnmatch
import sys
from pathlib import Path

import truststore

truststore.inject_into_ssl()  # pakai sertifikat Windows: jaringan/antivirus dengan inspeksi TLS bikin unduhan model gagal

from docling.document_converter import DocumentConverter


def main(folder, skip=()):
    out = Path(folder) / "md"
    out.mkdir(exist_ok=True)
    conv = DocumentConverter()
    for pdf in sorted(Path(folder).glob("*.pdf")):
        if any(fnmatch.fnmatch(pdf.name, s) for s in skip):
            print("lewati", pdf.name, flush=True)
            continue
        md = conv.convert(pdf).document.export_to_markdown()
        (out / f"{pdf.stem}.md").write_text(md, encoding="utf-8")
        print("ok", pdf.name, len(md), "karakter", flush=True)


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0], a[a.index("--skip") + 1:] if "--skip" in a else ())
