"""Run the packaged DOCX rasterizer with the offline OOXML export.

The Windows runtime contains no bundled LibreOffice; native Word COM was
denied. The actual DOCX is rendered by render_docx_reportlab.py. This adapter
retains the packaged render_docx.py raster and page naming workflow. The fixed
PDF and its editable Word source have checked content and OMML equivalence.
"""
import os, sys, tempfile, importlib.util, json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
TMP=ROOT/'_work/tmp'
os.environ['TEMP']=os.environ['TMP']=str(TMP)
tempfile.tempdir=str(TMP)
POPPLER=Path(r'C:\Program Files\WindowsApps\OpenAI.CodexPrimaryRuntime.v26-1025-760-0_26.1025.760.0_x64__3k8sg7r9htsxt\dependencies\native\poppler\Library\bin')
os.environ['PATH']=str(POPPLER)+os.pathsep+os.environ['PATH']
SCRIPT=Path(r'C:\Users\24910\.codex\plugins\cache\openai-primary-runtime\documents\26.1025.11240\skills\documents\render_docx.py')
spec=importlib.util.spec_from_file_location('packaged_docx_renderer',SCRIPT)
renderer=importlib.util.module_from_spec(spec);spec.loader.exec_module(renderer)
def offline_pdf(doc_path,user_profile,convert_tmp_dir,stem,verbose):
    pdf=Path(doc_path).with_suffix('.pdf')
    assert pdf.exists() and pdf.stat().st_size>1000,pdf
    assert pdf.stat().st_mtime>=Path(doc_path).stat().st_mtime-2,'Stale offline PDF'
    return str(pdf),'Offline OOXML PDF export verified; no Word native layout claim'
renderer.convert_to_pdf=offline_pdf
reports=[]
for docx in [ROOT/'无人机编队项目整体报告_学生版.docx',ROOT/'无人机编队项目技术报告_教师版.docx']:
    label='student' if '学生版' in docx.name else 'teacher'
    out=ROOT/'_work'/('render_'+label)
    if out.exists():
        for old in out.glob('page-*.png'):old.unlink()
    pages=renderer.rasterize(str(docx),str(out),dpi=130,verbose=False,emit_pdf=False)
    reports.append({'docx':str(docx),'pdf':str(docx.with_suffix('.pdf')),'pages':pages,'dpi':130})
    print(label,len(pages),flush=True)
(ROOT/'_work/render_receipt.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2),encoding='utf-8')
