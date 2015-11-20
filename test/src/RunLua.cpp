/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
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

#include "Lua/Basic.hpp"
#include "Lua/Log.hpp"
#include "Lua/RunFile.hpp"
#include "OS/Args.hpp"
#include "Util/Error.hxx"

extern "C" {
#include <lua.h>
}

#include <stdio.h>

static int
l_alert(lua_State *L)
{
  fprintf(stderr, "%s\n", lua_tostring(L, 1));
  return 0;
}

int main(int argc, char **argv)
{
  Args args(argc, argv, "FILE.lua");
  const auto path = args.ExpectNextPath();
  args.ExpectEnd();

  int result = EXIT_SUCCESS;

  lua_State *L = Lua::NewBasicState();
  Lua::InitLog(L);

  lua_register(L, "alert", l_alert);

  Error error;
  if (!Lua::RunFile(L, path, error)) {
    fprintf(stderr, "%s\n", error.GetMessage());
    result = EXIT_FAILURE;
  }

  lua_close(L);
  return result;
}