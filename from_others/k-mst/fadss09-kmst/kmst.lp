\Problem name: IloCplex

Minimize
 obj: 1000 id1 + 1000 id2 + 1000 id3 + 1000 id4 + 1000 id5 + 1000 id6
      + 1000 id7 + 1000 id8 + 1000 id9 + 1000 id10 + 778 id11 + 94 id12
      + 288 id13 + 948 id14 + 740 id15 + 902 id16 + 270 id17 + 578 id18
      + 772 id19 + 751 id20 + 395 id21 + 893 id22 + 640 id23 + 46 id24
      + 152 id25 + 714 id26 + 847 id27 + 765 id28 + 408 id29 + 67 id30
      + 931 id31 + 716 id32 + id311
Subject To
 id88:  id78 + id79 + id80 + id81 + id82 + id83 + id84 + id85 + id86  = 1
 id315: - id314 - 5 id77  = -1
 id318: id35 - id317  = 0
 id93:  id35 - id45 - id46 - id47 - id48 - id312  = 0
 id322: - 5 id78 - id321  = -1
 id325: id45 + id36 - id324  = 0
 id99:  id45 + id36 - id49 - id50 - id319  = 0
 id329: - 5 id79 - id328  = -1
 id332: id46 + id37 - id331  = 0
 id105: id46 + id37 - id51 - id52 - id326  = 0
 id336: - 5 id80 - id335  = -1
 id339: id47 + id49 + id38 - id338  = 0
 id111: id47 + id49 + id38 - id53 - id54 - id55 - id333  = 0
 id343: - 5 id81 - id342  = -1
 id346: id53 + id39 - id345  = 0
 id117: id53 + id39 - id56 - id57 - id340  = 0
 id350: - 5 id82 - id349  = -1
 id353: id51 + id40 - id352  = 0
 id123: id51 + id40 - id58 - id59 - id60 - id61 - id347  = 0
 id357: - 5 id83 - id356  = -1
 id360: id52 + id54 + id58 + id41 - id359  = 0
 id129: id52 + id54 + id58 + id41 - id62 - id63 - id64 - id354  = 0
 id364: - 5 id84 - id363  = -1
 id367: id48 + id59 + id62 + id42 - id366  = 0
 id135: id48 + id59 + id62 + id42 - id65 - id361  = 0
 id371: - 5 id85 - id370  = -1
 id374: id50 + id55 + id56 + id60 + id63 + id65 + id43 - id373  = 0
 id141: id50 + id55 + id56 + id60 + id63 + id65 + id43 - id66 - id368  = 0
 id378: - 5 id86 - id377  = -1
 id381: id57 + id61 + id64 + id66 + id44 - id380  = 0
 id147: id57 + id61 + id64 + id66 + id44 - id375  = 0
 id151: - 4 id1 + id35 <= 0
 id154: - 4 id2 + id36 <= 0
 id157: - 4 id3 + id37 <= 0
 id160: - 4 id4 + id38 <= 0
 id163: - 4 id5 + id39 <= 0
 id166: - 4 id6 + id40 <= 0
 id169: - 4 id7 + id41 <= 0
 id172: - 4 id8 + id42 <= 0
 id175: - 4 id9 + id43 <= 0
 id178: - 4 id10 + id44 <= 0
 id181: - 4 id11 + id45 <= 0
 id184: - 4 id12 + id46 <= 0
 id187: - 4 id13 + id47 <= 0
 id190: - 4 id14 + id48 <= 0
 id193: - 4 id15 + id49 <= 0
 id196: - 4 id16 + id50 <= 0
 id199: - 4 id17 + id51 <= 0
 id202: - 4 id18 + id52 <= 0
 id205: - 4 id19 + id53 <= 0
 id208: - 4 id20 + id54 <= 0
 id211: - 4 id21 + id55 <= 0
 id214: - 4 id22 + id56 <= 0
 id217: - 4 id23 + id57 <= 0
 id220: - 4 id24 + id58 <= 0
 id223: - 4 id25 + id59 <= 0
 id226: - 4 id26 + id60 <= 0
 id229: - 4 id27 + id61 <= 0
 id232: - 4 id28 + id62 <= 0
 id235: - 4 id29 + id63 <= 0
 id238: - 4 id30 + id64 <= 0
 id241: - 4 id31 + id65 <= 0
 id244: - 4 id32 + id66 <= 0
 id246: id1 - id35 <= 0
 id248: id2 - id36 <= 0
 id250: id3 - id37 <= 0
 id252: id4 - id38 <= 0
 id254: id5 - id39 <= 0
 id256: id6 - id40 <= 0
 id258: id7 - id41 <= 0
 id260: id8 - id42 <= 0
 id262: id9 - id43 <= 0
 id264: id10 - id44 <= 0
 id266: id11 - id45 <= 0
 id268: id12 - id46 <= 0
 id270: id13 - id47 <= 0
 id272: id14 - id48 <= 0
 id274: id15 - id49 <= 0
 id276: id16 - id50 <= 0
 id278: id17 - id51 <= 0
 id280: id18 - id52 <= 0
 id282: id19 - id53 <= 0
 id284: id20 - id54 <= 0
 id286: id21 - id55 <= 0
 id288: id22 - id56 <= 0
 id290: id23 - id57 <= 0
 id292: id24 - id58 <= 0
 id294: id25 - id59 <= 0
 id296: id26 - id60 <= 0
 id298: id27 - id61 <= 0
 id300: id28 - id62 <= 0
 id302: id29 - id63 <= 0
 id304: id30 - id64 <= 0
 id306: id31 - id65 <= 0
 id308: id32 - id66 <= 0
 id310: id1 + id2 + id3 + id4 + id5 + id6 + id7 + id8 + id9 + id10 + id11
        + id12 + id13 + id14 + id15 + id16 + id17 + id18 + id19 + id20 + id21
        + id22 + id23 + id24 + id25 + id26 + id27 + id28 + id29 + id30 + id31
        + id32  = 4
 c97:   - id312 + id314 >= 0
 c98:   - id312 + id317 >= 0
 c99:   x106 + x107 >= 1
 c100:  - id319 + id321 >= 0
 c101:  - id319 + id324 >= 0
 c102:  x108 + x109 >= 1
 c103:  - id326 + id328 >= 0
 c104:  - id326 + id331 >= 0
 c105:  x110 + x111 >= 1
 c106:  - id333 + id335 >= 0
 c107:  - id333 + id338 >= 0
 c108:  x112 + x113 >= 1
 c109:  - id340 + id342 >= 0
 c110:  - id340 + id345 >= 0
 c111:  x114 + x115 >= 1
 c112:  - id347 + id349 >= 0
 c113:  - id347 + id352 >= 0
 c114:  x116 + x117 >= 1
 c115:  - id354 + id356 >= 0
 c116:  - id354 + id359 >= 0
 c117:  x118 + x119 >= 1
 c118:  - id361 + id363 >= 0
 c119:  - id361 + id366 >= 0
 c120:  x120 + x121 >= 1
 c121:  - id368 + id370 >= 0
 c122:  - id368 + id373 >= 0
 c123:  x122 + x123 >= 1
 c124:  - id375 + id377 >= 0
 c125:  - id375 + id380 >= 0
 c126:  x124 + x125 >= 1
 i1:  x106 = 1 <-> id314 - id312 <= 0
 i2:  x107 = 1 <-> id317 - id312 <= 0
 i3:  x108 = 1 <-> id321 - id319 <= 0
 i4:  x109 = 1 <-> id324 - id319 <= 0
 i5:  x110 = 1 <-> id328 - id326 <= 0
 i6:  x111 = 1 <-> id331 - id326 <= 0
 i7:  x112 = 1 <-> id335 - id333 <= 0
 i8:  x113 = 1 <-> id338 - id333 <= 0
 i9:  x114 = 1 <-> id342 - id340 <= 0
 i10: x115 = 1 <-> id345 - id340 <= 0
 i11: x116 = 1 <-> id349 - id347 <= 0
 i12: x117 = 1 <-> id352 - id347 <= 0
 i13: x118 = 1 <-> id356 - id354 <= 0
 i14: x119 = 1 <-> id359 - id354 <= 0
 i15: x120 = 1 <-> id363 - id361 <= 0
 i16: x121 = 1 <-> id366 - id361 <= 0
 i17: x122 = 1 <-> id370 - id368 <= 0
 i18: x123 = 1 <-> id373 - id368 <= 0
 i19: x124 = 1 <-> id377 - id375 <= 0
 i20: x125 = 1 <-> id380 - id375 <= 0
Bounds
 0 <= id1 <= 1
 0 <= id2 <= 1
 0 <= id3 <= 1
 0 <= id4 <= 1
 0 <= id5 <= 1
 0 <= id6 <= 1
 0 <= id7 <= 1
 0 <= id8 <= 1
 0 <= id9 <= 1
 0 <= id10 <= 1
 0 <= id11 <= 1
 0 <= id12 <= 1
 0 <= id13 <= 1
 0 <= id14 <= 1
 0 <= id15 <= 1
 0 <= id16 <= 1
 0 <= id17 <= 1
 0 <= id18 <= 1
 0 <= id19 <= 1
 0 <= id20 <= 1
 0 <= id21 <= 1
 0 <= id22 <= 1
 0 <= id23 <= 1
 0 <= id24 <= 1
 0 <= id25 <= 1
 0 <= id26 <= 1
 0 <= id27 <= 1
 0 <= id28 <= 1
 0 <= id29 <= 1
 0 <= id30 <= 1
 0 <= id31 <= 1
 0 <= id32 <= 1
      id311 = 0
 0 <= id78 <= 1
 0 <= id79 <= 1
 0 <= id80 <= 1
 0 <= id81 <= 1
 0 <= id82 <= 1
 0 <= id83 <= 1
 0 <= id84 <= 1
 0 <= id85 <= 1
 0 <= id86 <= 1
 0 <= id35 <= 4
 0 <= id45 <= 4
 0 <= id46 <= 4
 0 <= id47 <= 4
 0 <= id48 <= 4
      id312 Free
      id314 Free
 0 <= id77 <= 1
      id317 Free
 0 <= id36 <= 4
 0 <= id49 <= 4
 0 <= id50 <= 4
      id319 Free
      id321 Free
      id324 Free
 0 <= id37 <= 4
 0 <= id51 <= 4
 0 <= id52 <= 4
      id326 Free
      id328 Free
      id331 Free
 0 <= id38 <= 4
 0 <= id53 <= 4
 0 <= id54 <= 4
 0 <= id55 <= 4
      id333 Free
      id335 Free
      id338 Free
 0 <= id39 <= 4
 0 <= id56 <= 4
 0 <= id57 <= 4
      id340 Free
      id342 Free
      id345 Free
 0 <= id40 <= 4
 0 <= id58 <= 4
 0 <= id59 <= 4
 0 <= id60 <= 4
 0 <= id61 <= 4
      id347 Free
      id349 Free
      id352 Free
 0 <= id41 <= 4
 0 <= id62 <= 4
 0 <= id63 <= 4
 0 <= id64 <= 4
      id354 Free
      id356 Free
      id359 Free
 0 <= id42 <= 4
 0 <= id65 <= 4
      id361 Free
      id363 Free
      id366 Free
 0 <= id43 <= 4
 0 <= id66 <= 4
      id368 Free
      id370 Free
      id373 Free
 0 <= id44 <= 4
      id375 Free
      id377 Free
      id380 Free
 0 <= x106 <= 1
 0 <= x107 <= 1
 0 <= x108 <= 1
 0 <= x109 <= 1
 0 <= x110 <= 1
 0 <= x111 <= 1
 0 <= x112 <= 1
 0 <= x113 <= 1
 0 <= x114 <= 1
 0 <= x115 <= 1
 0 <= x116 <= 1
 0 <= x117 <= 1
 0 <= x118 <= 1
 0 <= x119 <= 1
 0 <= x120 <= 1
 0 <= x121 <= 1
 0 <= x122 <= 1
 0 <= x123 <= 1
 0 <= x124 <= 1
 0 <= x125 <= 1
Binaries
 id78  id79  id80  id81  id82  id83  id84  id85  id86  id77  x106  x107  x108 
 x109  x110  x111  x112  x113  x114  x115  x116  x117  x118  x119  x120  x121 
 x122  x123  x124  x125 
End
