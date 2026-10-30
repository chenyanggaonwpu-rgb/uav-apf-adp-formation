"""Offline OOXML renderer for this report pair.

Reads the actual DOCX paragraph/table/image content. Native OMML is checked
against the exact MathML source used to author it before mathematical layout.
The exported PDF is the fixed-layout edition; Word remains editable.
Native Word COM returned E_ACCESSDENIED, the bundled runtime has no LibreOffice,
and desktop automation initialization failed. No desktop safety setting is changed.
"""
from __future__ import annotations
import os, sys, io, re, copy, json, hashlib
from pathlib import Path
from xml.sax.saxutils import escape
from lxml import etree
from docx import Document
from docx.oxml.ns import qn
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle, Image as RLImage, Flowable
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.colors import HexColor, black, white
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from PIL import Image
import build_reports as source

ROOT=Path(__file__).resolve().parent
for name,path,index in [
 ('BodyCN',r'C:\Windows\Fonts\simsun.ttc',0),
 ('HeadCN',r'C:\Windows\Fonts\simhei.ttf',0),
 ('MathRegular',r'C:\Windows\Fonts\cambria.ttc',0),
 ('MathSymbol',r'C:\Windows\Fonts\cambria.ttc',1),
 ('MathItalic',r'C:\Windows\Fonts\cambriai.ttf',0),
]:
    pdfmetrics.registerFont(TTFont(name,path,subfontIndex=index))
pdfmetrics.registerFontFamily('BodyCN',normal='BodyCN',bold='HeadCN',italic='BodyCN',boldItalic='HeadCN')
pdfmetrics.registerFontFamily('HeadCN',normal='HeadCN',bold='HeadCN',italic='HeadCN',boldItalic='HeadCN')

class MathBox:
    def __init__(self,w,a,d,ops=None):self.w,self.a,self.d,self.ops=w,a,d,ops or []
    def shifted(self,x,y):
        return [(kind,(args[0]+x,args[1]+y,*args[2:])) if kind=='text' else
                (kind,(args[0]+x,args[1]+y,args[2]+x,args[3]+y,*args[4:])) for kind,args in self.ops]

def glyph(text,size,font='MathSymbol'):
    text=text.replace('\u2061','').replace('\u2062','').replace('\u2063',',')
    w=pdfmetrics.stringWidth(text,font,size)
    return MathBox(w,.78*size,.22*size,[('text',(0,0,text,font,size))])

def horizontal(boxes,gap=0):
    x=0;ops=[];a=d=0
    for b in boxes:
        ops.extend(b.shifted(x,0));x+=b.w+gap;a=max(a,b.a);d=max(d,b.d)
    return MathBox(max(0,x-gap),a,d,ops)

def math_box(node,size=12):
    tag=etree.QName(node).localname
    kids=list(node)
    if tag in ('mi','mn','mo','mtext'):
        text=''.join(node.itertext())
        font='MathItalic' if tag=='mi' and len(text)<=2 and all(ord(ch)<1024 for ch in text) else 'MathSymbol'
        if tag=='mtext' and any(ord(ch)>0x3000 for ch in text):font='BodyCN'
        b=glyph(text,size,font)
        if tag=='mo' and text in ('=','+','−','≤','≥',','):
            pad=size*.15;b=horizontal([MathBox(pad,0,0),b,MathBox(pad,0,0)])
        return b
    if tag in ('math','mrow','mstyle','mtd','semantics'):
        return horizontal([math_box(k,size) for k in kids if etree.QName(k).localname not in ('annotation','annotation-xml')])
    if tag=='msub':
        base,sub=math_box(kids[0],size),math_box(kids[1],size*.68)
        sy=-size*.29
        return MathBox(base.w+sub.w,base.a,max(base.d,sub.d-sy),base.ops+sub.shifted(base.w,sy))
    if tag=='msup':
        base,up=math_box(kids[0],size),math_box(kids[1],size*.68)
        sy=max(size*.45,base.a*.65)
        return MathBox(base.w+up.w,max(base.a,sy+up.a),base.d,base.ops+up.shifted(base.w,sy))
    if tag=='msubsup':
        base,lo,hi=math_box(kids[0],size),math_box(kids[1],size*.68),math_box(kids[2],size*.68)
        return MathBox(base.w+max(lo.w,hi.w),max(base.a,.48*size+hi.a),max(base.d,.3*size+lo.d),base.ops+lo.shifted(base.w,-.3*size)+hi.shifted(base.w,.48*size))
    if tag=='mfrac':
        num,den=math_box(kids[0],size*.91),math_box(kids[1],size*.91)
        w=max(num.w,den.w)+8;rule=size*.23
        ny=rule+3+num.d;dy=rule-3-den.a
        return MathBox(w,ny+num.a,den.d-dy,num.shifted((w-num.w)/2,ny)+den.shifted((w-den.w)/2,dy)+[('line',(0,rule,w,rule,.6))])
    if tag=='msqrt':
        b=horizontal([math_box(k,size) for k in kids]);s=glyph('√',size*1.15)
        x=s.w*.90;top=b.a+2
        return MathBox(x+b.w+2,top+1,b.d,s.ops+b.shifted(x,0)+[('line',(x,top,x+b.w+1,top,.65))])
    if tag=='mfenced':
        b=horizontal([math_box(k,size) for k in kids])
        if node.get('open')=='[' and b.a+b.d>size*1.5:
            a,d=b.a+1,b.d+1
            left=MathBox(6,a,d,[('line',(5,a,1,a,.7)),('line',(1,a,1,-d,.7)),('line',(1,-d,5,-d,.7))])
            right=MathBox(6,a,d,[('line',(1,a,5,a,.7)),('line',(5,a,5,-d,.7)),('line',(5,-d,1,-d,.7))])
            return horizontal([left,b,right])
        return horizontal([glyph(node.get('open','('),size),b,glyph(node.get('close',')'),size)])
    if tag in ('munderover','munder','mover'):
        b=math_box(kids[0],size*1.3)
        lo=math_box(kids[1],size*.60) if tag!='mover' else None
        hi=math_box(kids[2] if tag=='munderover' else kids[1],size*.60) if tag!='munder' else None
        w=max([b.w]+([lo.w] if lo else [])+([hi.w] if hi else []))+3
        ops=b.shifted((w-b.w)/2,0);a,d=b.a,b.d
        if lo:
            y=-b.d-1-lo.a;ops+=lo.shifted((w-lo.w)/2,y);d=-y+lo.d
        if hi:
            y=b.a+1+hi.d;ops+=hi.shifted((w-hi.w)/2,y);a=y+hi.a
        return MathBox(w,a,d,ops)
    if tag=='mtable':
        rows=[[math_box(cell,size) for cell in tr] for tr in kids]
        cols=max(map(len,rows),default=0);widths=[max((r[i].w if i<len(r) else 0) for r in rows) for i in range(cols)]
        gap=size*.7;heights=[max((b.a+b.d for b in r),default=size)+size*.40 for r in rows]
        total=sum(heights);top=total/2+size*.2;ops=[];cur=top
        left=node.get('columnalign')=='left'
        for rr,rh in zip(rows,heights):
            ra=max(b.a for b in rr);baseline=cur-ra-size*.12;x=0
            for i,b in enumerate(rr):
                offset=0 if left else (widths[i]-b.w)/2
                ops+=b.shifted(x+offset,baseline);x+=widths[i]+gap
            cur-=rh
        return MathBox(sum(widths)+gap*max(0,cols-1),top,-cur,ops)
    if tag=='mspace':return MathBox(size*.25,0,0)
    raise ValueError('Unsupported mathematical element '+tag)

class MathFlow(Flowable):
    def __init__(self,xml):
        Flowable.__init__(self)
        self.box=math_box(etree.fromstring(('<math xmlns="http://www.w3.org/1998/Math/MathML">'+xml+'</math>').encode()),12)
        self.scale=min(1,493/max(1,self.box.w));self.width=504;self.height=(self.box.a+self.box.d)*self.scale+12
    def draw(self):
        c=self.canv;c.saveState();c.translate((504-self.box.w*self.scale)/2,6+self.box.d*self.scale);c.scale(self.scale,self.scale)
        for kind,args in self.box.ops:
            if kind=='text':
                x,y,text,font,size=args;c.setFont(font,size);c.setFillColor(black);c.drawString(x,y,text)
            else:
                x1,y1,x2,y2,width=args;c.setLineWidth(width);c.setStrokeColor(black);c.line(x1,y1,x2,y2)
        c.restoreState()

class ReportDoc(SimpleDocTemplate):
    def __init__(self,*args,**kwargs):super().__init__(*args,**kwargs);self.titles=[]
    def afterFlowable(self,flowable):
        if isinstance(flowable,Paragraph) and flowable.style.name in ('Title','Heading1'):
            key='heading_'+str(len(self.titles));text=flowable.getPlainText()
            self.canv.bookmarkPage(key);self.canv.addOutlineEntry(text,key,0,False)
            self.titles.append({'title':text,'page':self.page})

def descendants_text(node):return ''.join(node.itertext())

def clean_math(node):
    out=copy.deepcopy(node)
    for e in list(out.iter(qn('w:rPr'))):e.getparent().remove(e)
    return etree.tostring(out,method='c14n',exclusive=True)

def xmlruns(p,doc):
    chunks=[]
    for child in p:
        if child.tag==qn('w:r'):
            text=''.join(child.xpath('.//w:t/text()',namespaces=child.nsmap)) if not hasattr(child,'xpath') else ''.join(t.text or '' for t in child.iter(qn('w:t')))
            if not text:continue
            pr=child.find(qn('w:rPr'));size=None;color=None;bold=False
            if pr is not None:
                sz=pr.find(qn('w:sz'));size=float(sz.get(qn('w:val')))/2 if sz is not None else None
                cc=pr.find(qn('w:color'));color=cc.get(qn('w:val')) if cc is not None else None
                bb=pr.find(qn('w:b'));bold=bb is not None and bb.get(qn('w:val'),'true') not in ('false','0')
            s=escape(text)
            if bold:s='<b>'+s+'</b>'
            attrs=(f' size="{size:g}"' if size else '')+(f' color="#{color}"' if color and color!='auto' else '')
            if attrs:s='<font'+attrs+'>'+s+'</font>'
            chunks.append(s)
        elif child.tag==qn('w:hyperlink'):
            rid=child.get(qn('r:id'));target=doc.part.rels[rid].target_ref
            label=''.join(t.text or '' for t in child.iter(qn('w:t')))
            chunks.append('<link href="'+escape(target,{'"':'&quot;'})+'" color="#285477">'+escape(label)+'</link>')
    return ''.join(chunks)

def render(docpath,mdpath):
    doc=Document(docpath);teacher='教师版' in docpath.name;body=11 if teacher else 11.5
    styles={
        'Normal':ParagraphStyle('Body',fontName='BodyCN',fontSize=body,leading=body*1.25,spaceAfter=7,wordWrap='CJK',allowWidows=0,allowOrphans=0),
        'Title':ParagraphStyle('Title',fontName='HeadCN',fontSize=23,leading=31,spaceAfter=12,wordWrap='CJK',keepWithNext=True),
        'Heading1':ParagraphStyle('Heading1',fontName='HeadCN',fontSize=16,leading=22,spaceBefore=4,spaceAfter=10,wordWrap='CJK',keepWithNext=True),
        'Caption':ParagraphStyle('Caption',fontName='BodyCN',fontSize=9,leading=12,spaceAfter=9,alignment=TA_CENTER,wordWrap='CJK'),
        'Small':ParagraphStyle('Small',fontName='BodyCN',fontSize=9.4,leading=12,spaceAfter=5,wordWrap='CJK'),
    }
    keys=re.findall(r'^@eq (\S+)',mdpath.read_text(encoding='utf-8'),re.M)
    ei=0;story=[];para_count=table_count=image_count=0
    for el in doc.element.body:
        if el.tag==qn('w:p'):
            if any(b.get(qn('w:type'))=='page' for b in el.iter(qn('w:br'))):story.append(PageBreak());continue
            maths=list(el.iter(qn('m:oMathPara')))
            if not maths:maths=[m for m in el if m.tag==qn('m:oMath')]
            if maths:
                key=keys[ei];ei+=1
                expected=source.XSL(etree.fromstring(('<math xmlns="http://www.w3.org/1998/Math/MathML" display="block">'+source.EQS[key]+'</math>').encode())).getroot()
                assert clean_math(maths[0])==clean_math(expected),'OMML mismatch '+key
                story.append(MathFlow(source.EQS[key]));continue
            drawings=list(el.iter(qn('w:drawing')))
            if drawings:
                for dr in drawings:
                    blip=next(dr.iter('{http://schemas.openxmlformats.org/drawingml/2006/main}blip'))
                    rid=blip.get(qn('r:embed'));blob=doc.part.related_parts[rid].blob
                    im=Image.open(io.BytesIO(blob));width=6.8*72;height=width*im.height/im.width
                    extent=next(dr.iter('{http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing}extent'),None)
                    if extent is not None:width=int(extent.get('cx'))/12700;height=int(extent.get('cy'))/12700
                    flow=RLImage(io.BytesIO(blob),width=width,height=height);flow.hAlign='CENTER';flow.spaceAfter=3
                    flow.keepWithNext=True;story.append(flow);image_count+=1
                continue
            text=xmlruns(el,doc)
            if not text:
                story.append(Spacer(1,4));continue
            pr=el.find(qn('w:pPr'));style='Normal';align=None
            if pr is not None:
                ps=pr.find(qn('w:pStyle'));style=ps.get(qn('w:val')) if ps is not None else 'Normal'
                jc=pr.find(qn('w:jc'));align=jc.get(qn('w:val')) if jc is not None else None
            chosen=styles.get(style,styles['Normal'])
            if align=='center':chosen=styles['Caption']
            if text.startswith('<font size="9') or re.match(r'^(<font[^>]*>)?E\d\d',text):chosen=styles['Small']
            story.append(Paragraph(text,chosen));para_count+=1
        elif el.tag==qn('w:tbl'):
            rows=[]
            for tr in el.findall(qn('w:tr')):
                row=[]
                for cell in tr.findall(qn('w:tc')):
                    text=''.join(t.text or '' for t in cell.iter(qn('w:t')))
                    row.append(text)
                rows.append(row)
            n=len(rows[0]);grid=el.find(qn('w:tblGrid'))
            widths=[int(c.get(qn('w:w')))/20 for c in grid] if grid is not None else [504/n]*n
            factor=504/sum(widths);widths=[w*factor for w in widths]
            pdata=[]
            for ri,rr in enumerate(rows):
                cells=[]
                for ci,text in enumerate(rr):
                    centered=ci>0 and (len(text)<17 or re.match(r'^[0-9−±≤≥—]',text))
                    style=ParagraphStyle('cell',fontName='HeadCN' if ri==0 else 'BodyCN',fontSize=10.2 if n<=4 else 9.5,leading=12.6 if n<=4 else 11.6,textColor=white if ri==0 else black,wordWrap='CJK',alignment=TA_CENTER if centered else TA_LEFT)
                    safe_text=escape(text).replace('²','<super>2</super>').replace('³','<super>3</super>')
                    cells.append(Paragraph(safe_text,style))
                pdata.append(cells)
            tbl=Table(pdata,colWidths=widths,repeatRows=1,hAlign='CENTER')
            cmds=[('GRID',(0,0),(-1,-1),.5,HexColor('#D9D9D9')),('VALIGN',(0,0),(-1,-1),'MIDDLE'),('BACKGROUND',(0,0),(-1,0),HexColor('#354C5B')),('LEFTPADDING',(0,0),(-1,-1),4.25),('RIGHTPADDING',(0,0),(-1,-1),4.25),('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),4)]
            for ri in range(1,len(rows)):
                if ri%2:cmds.append(('BACKGROUND',(0,ri),(-1,ri),HexColor('#F2F5F7')))
            tbl.setStyle(TableStyle(cmds));tbl.spaceAfter=5;story.append(tbl);table_count+=1
    assert ei==len(keys)
    out=docpath.with_suffix('.pdf')
    document=ReportDoc(str(out),pagesize=(612,792),rightMargin=54,leftMargin=54,topMargin=48.24,bottomMargin=46.8,title=doc.core_properties.title,author=doc.core_properties.author)
    def footer(c,d):
        c.saveState();c.setFillColor(HexColor('#666666'));c.setFont('BodyCN',9);c.drawCentredString(306,21,str(d.page));c.restoreState()
    document.build(story,onFirstPage=footer,onLaterPages=footer)
    receipt={'docx':str(docpath),'pdf':str(out),'renderer':'ReportLab OOXML and structured MathML renderer','native_omml_equivalence':True,'equations':ei,'paragraphs':para_count,'tables':table_count,'images':image_count,'headings':document.titles,'pages':max(t['page'] for t in document.titles),'docx_sha256':hashlib.sha256(docpath.read_bytes()).hexdigest()}
    print(json.dumps(receipt|{'headings':len(document.titles)},ensure_ascii=False),flush=True)
    return receipt

if __name__=='__main__':
    receipts=[]
    for name,md in [('无人机编队项目整体报告_学生版','学生版_正文.md'),('无人机编队项目技术报告_教师版','教师版_正文.md')]:receipts.append(render(ROOT/(name+'.docx'),ROOT/md))
    (ROOT/'_work/offline_export_receipt.json').write_text(json.dumps(receipts,ensure_ascii=False,indent=2),encoding='utf-8')
