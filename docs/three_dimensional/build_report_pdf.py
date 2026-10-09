"""Render the editable 3-D stage report Markdown to a Chinese A4 PDF.

Requires ReportLab and Pillow. Usage:
    python build_report_pdf.py TECHNICAL_REPORT.md 三维阶段技术报告.pdf
This file changes layout only; experimental numbers belong in the Markdown.
"""

from __future__ import annotations

import html
import re
import sys
from pathlib import Path

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    Image,
    PageBreak,
    PageTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
)


BLUE = colors.HexColor("#173b63")
INK = colors.HexColor("#1b2531")
MUTED = colors.HexColor("#526274")
PALE = colors.HexColor("#eef4f8")
PAGE_W, PAGE_H = A4
LEFT = RIGHT = 21 * mm
TOP = 22 * mm
BOTTOM = 21 * mm
CONTENT_W = PAGE_W - LEFT - RIGHT


def register_chinese_font() -> None:
    candidates = [
        Path("C:/Windows/Fonts/simhei.ttf"),
        Path("C:/Windows/Fonts/msyh.ttc"),
    ]
    for font in candidates:
        if font.exists():
            try:
                pdfmetrics.registerFont(TTFont("Chinese", str(font)))
                return
            except Exception:
                pass
    raise RuntimeError("A Chinese TrueType font is required to render the report.")


def inline_markup(text: str) -> str:
    # SimHei does not contain every Unicode math glyph; keep formulas legible.
    for source, replacement in (
        ("−", "-"), ("–", "-"), ("—", "-"),
        ("²", "^2"), ("³", "^3"), ("⁻", "-"), ("⁶", "6"),
        ("√", "sqrt"), ("≈", "~"), ("Δ", "Delta"),
        ("→", "->"), ("≤", "<="), ("≥", ">="),
    ):
        text = text.replace(source, replacement)
    text = html.escape(text, quote=False)
    text = re.sub(r"\[([^]]+)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", text)
    text = re.sub(r"`([^`]+)`", r"<font color='#173b63'>\1</font>", text)
    return text


def make_styles() -> dict[str, ParagraphStyle]:
    common = dict(fontName="Chinese", textColor=INK, wordWrap="CJK")
    return {
        "title": ParagraphStyle(
            "title", fontSize=25, leading=36, alignment=TA_CENTER,
            spaceAfter=16 * mm, textColor=BLUE, **{k: v for k, v in common.items() if k != "textColor"}
        ),
        "subtitle": ParagraphStyle(
            "subtitle", fontSize=12, leading=22, alignment=TA_CENTER,
            textColor=MUTED, spaceAfter=7 * mm, **{k: v for k, v in common.items() if k != "textColor"}
        ),
        "h1": ParagraphStyle(
            "h1", fontSize=15, leading=23, textColor=BLUE,
            spaceBefore=10 * mm, spaceAfter=4 * mm,
            keepWithNext=True, **{k: v for k, v in common.items() if k != "textColor"}
        ),
        "h2": ParagraphStyle(
            "h2", fontSize=11.5, leading=18, textColor=BLUE,
            spaceBefore=5 * mm, spaceAfter=2.5 * mm,
            keepWithNext=True, **{k: v for k, v in common.items() if k != "textColor"}
        ),
        "body": ParagraphStyle(
            "body", fontSize=9.3, leading=16.2, spaceAfter=3 * mm,
            alignment=TA_LEFT, **common
        ),
        "bullet": ParagraphStyle(
            "bullet", fontSize=9.3, leading=16.2, leftIndent=5 * mm,
            firstLineIndent=-3 * mm, spaceAfter=1.5 * mm, **common
        ),
        "table_head": ParagraphStyle(
            "table_head", fontSize=8.4, leading=13, textColor=colors.white,
            **{k: v for k, v in common.items() if k != "textColor"}
        ),
        "table_cell": ParagraphStyle(
            "table_cell", fontSize=8.2, leading=12.8, **common
        ),
        "caption": ParagraphStyle(
            "caption", fontSize=8.5, leading=13, alignment=TA_CENTER,
            textColor=MUTED, spaceBefore=2 * mm, spaceAfter=5 * mm,
            **{k: v for k, v in common.items() if k != "textColor"}
        ),
    }


def table_flowable(lines: list[str], styles: dict[str, ParagraphStyle]) -> Table:
    parsed = [[cell.strip() for cell in line.strip().strip("|").split("|")] for line in lines]
    parsed = [row for row in parsed if not all(re.fullmatch(r":?-{2,}:?", c) for c in row)]
    cols = len(parsed[0])
    if any(len(row) != cols for row in parsed):
        raise ValueError("Inconsistent Markdown table column count")
    weights = []
    for col in range(cols):
        longest = max(len(row[col]) for row in parsed)
        weights.append(max(9, min(38, longest)))
    widths = [CONTENT_W * weight / sum(weights) for weight in weights]
    cells = [
        [
            Paragraph(inline_markup(cell), styles["table_head" if index == 0 else "table_cell"])
            for cell in row
        ]
        for index, row in enumerate(parsed)
    ]
    table = Table(cells, colWidths=widths, repeatRows=1, hAlign="LEFT")
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), BLUE),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, PALE]),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("LINEBELOW", (0, 0), (-1, 0), 0.7, BLUE),
        ("LINEBELOW", (0, -1), (-1, -1), 0.4, colors.HexColor("#b6c4ce")),
    ]))
    return table


def image_flowables(line: str, parent: Path, styles: dict[str, ParagraphStyle]) -> list:
    match = re.fullmatch(r"!\[([^]]*)\]\(([^)]+)\)", line)
    if not match:
        raise ValueError(f"Invalid image reference: {line}")
    caption, relative = match.groups()
    path = (parent / relative).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    width_px, height_px = PILImage.open(path).size
    scale = min(CONTENT_W / width_px, 163 * mm / height_px)
    picture = Image(str(path), width=width_px * scale, height=height_px * scale)
    return [Spacer(1, 3 * mm), picture, Paragraph(inline_markup(caption), styles["caption"])]


def parse_markdown(path: Path, styles: dict[str, ParagraphStyle]) -> list:
    lines = path.read_text(encoding="utf-8").splitlines()
    story = []
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if not line:
            i += 1
            continue
        if line.startswith("# "):
            i += 1  # The document title is placed on the cover.
            continue
        if line == "<!-- PAGEBREAK -->":
            story.append(PageBreak())
            i += 1
            continue
        if line.startswith("## "):
            story.append(Paragraph(inline_markup(line[3:]), styles["h1"]))
            i += 1
            continue
        if line.startswith("### "):
            story.append(Paragraph(inline_markup(line[4:]), styles["h2"]))
            i += 1
            continue
        if line.startswith("| "):
            table_lines = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                table_lines.append(lines[i])
                i += 1
            story.extend([table_flowable(table_lines, styles), Spacer(1, 3 * mm)])
            continue
        if line.startswith("!["):
            story.extend(image_flowables(line, path.parent, styles))
            i += 1
            continue
        if line.startswith("- "):
            story.append(Paragraph("• " + inline_markup(line[2:]), styles["bullet"]))
            i += 1
            continue
        paragraph = [line]
        i += 1
        while i < len(lines) and lines[i].strip() and not lines[i].lstrip().startswith(
            ("#", "|", "- ", "![", "<!-- PAGEBREAK -->")
        ):
            paragraph.append(lines[i].strip())
            i += 1
        story.append(Paragraph(inline_markup(" ".join(paragraph)), styles["body"]))
    return story


def decorate(canvas, doc) -> None:
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#ccd8e2"))
    canvas.setLineWidth(0.5)
    canvas.line(LEFT, PAGE_H - 14 * mm, PAGE_W - RIGHT, PAGE_H - 14 * mm)
    canvas.setFont("Chinese", 8)
    canvas.setFillColor(MUTED)
    canvas.drawString(LEFT, PAGE_H - 11 * mm, "无人机编队智能避障 · 三维阶段技术报告")
    canvas.drawRightString(PAGE_W - RIGHT, 11 * mm, f"第 {doc.page} 页")
    canvas.restoreState()


def build(source: Path, target: Path) -> None:
    register_chinese_font()
    styles = make_styles()
    document = BaseDocTemplate(
        str(target), pagesize=A4, leftMargin=LEFT, rightMargin=RIGHT,
        topMargin=TOP, bottomMargin=BOTTOM, title="无人机编队智能避障三维阶段技术报告",
        author="UAV APF ADP Formation project",
    )
    frame = Frame(LEFT, BOTTOM, CONTENT_W, PAGE_H - TOP - BOTTOM,
                  leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
    document.addPageTemplates(PageTemplate(id="body", frames=[frame], onPage=decorate))
    story = [
        Spacer(1, 48 * mm),
        Paragraph("无人机编队智能避障", styles["title"]),
        Paragraph("三维有限高度层改造与阶段验证报告", styles["title"]),
        Paragraph("版本 2.0.0 · MATLAB R2024a · 2026 年 10 月 3 日", styles["subtitle"]),
        Spacer(1, 9 * mm),
        Paragraph(
            "围绕可解释性、实时性、鲁棒性与轻量性的程序实现、实验数据和适用范围",
            styles["subtitle"],
        ),
        PageBreak(),
    ]
    story.extend(parse_markdown(source, styles))
    target.parent.mkdir(parents=True, exist_ok=True)
    document.build(story)


if __name__ == "__main__":
    build(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
