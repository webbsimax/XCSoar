/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Screen/Font.hpp"
#include "Screen/Debug.hpp"
#include "Screen/Custom/Files.hpp"

#include <assert.h>

static const char *font_path;
static const char *monospace_font_path;

void
Font::Initialise()
{
  font_path = FindDefaultFont();

  monospace_font_path = FindDefaultMonospaceFont();
  if (monospace_font_path == NULL)
    monospace_font_path = font_path;
}

bool
Font::_set(const char *file, UPixelScalar ptsize, bool bold, bool italic)
{
  assert(IsScreenInitialized());

  Reset();

  font = TTF_OpenFont(file, ptsize);
  if (font == NULL)
    return false;

  int style = TTF_STYLE_NORMAL;
  if (bold)
    style |= TTF_STYLE_BOLD;
  if (italic)
    style |= TTF_STYLE_ITALIC;
  TTF_SetFontStyle(font, style);

  CalculateHeights();

  return true;
}

bool
Font::Set(const TCHAR *facename, UPixelScalar height, bool bold, bool italic)
{
  LOGFONT lf;
  lf.lfWeight = bold ? 700 : 500;
  lf.lfHeight = height;
  lf.lfItalic = italic;
  lf.lfPitchAndFamily = FF_DONTCARE | VARIABLE_PITCH;
  return Set(lf);
}

bool
Font::Set(const LOGFONT &log_font)
{
  assert(IsScreenInitialized());

  const char *path = (log_font.lfPitchAndFamily & 0x03) == FIXED_PITCH
    ? monospace_font_path
    : font_path;

  if (path == NULL)
    return false;

  return _set(path, log_font.lfHeight > 0 ? log_font.lfHeight : 10,
              log_font.lfWeight >= 700,
              log_font.lfItalic);
}

void
Font::CalculateHeights()
{
  height = TTF_FontHeight(font);
  ascent_height = TTF_FontAscent(font);

  int miny, maxy;
  TTF_GlyphMetrics(font, 'M', NULL, NULL, &miny, &maxy, NULL);

  capital_height = maxy - miny + 1;
}

void
Font::Reset()
{
  assert(!IsDefined() || IsScreenInitialized());

  if (font != NULL) {
    assert(IsScreenInitialized());

    TTF_CloseFont(font);
    font = NULL;
  }
}

PixelSize
Font::TextSize(const TCHAR *text) const
{
  int ret, w, h;
#ifdef UNICODE
  ret = ::TTF_SizeUNICODE(font, (const Uint16 *)text, &w, &h);
#else
  ret = ::TTF_SizeUTF8(font, text, &w, &h);
#endif
  PixelSize size;
  if (ret == 0) {
    size.cx = w;
    size.cy = h;
  } else
    size.cx = size.cy = 0;
  return size;
}
