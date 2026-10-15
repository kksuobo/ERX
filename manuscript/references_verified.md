# Reference verification report

Method: Crossref field-level check, using `api.crossref.org/works/<DOI>` and the
`query.bibliographic` search endpoint. Verified 2026-10-13.

## Summary (23 entries)

- ✅ Verified: 23
- ❌ Needs fix (corrected before finalising): 1 — reference 6 (see below)
- ❓ Unverifiable: 0

## ❌ Corrected entry

| # | Problem | Was | Now (verified) | Source |
|---|---|---|---|---|
| 6 | Wrong venue, year and author list | Rösmann C, Feiten W, Wösch T, Hoffmann F, Bertram T. Trajectory modification considering dynamic constraints of autonomous robots. *Robotics: Science and Systems*, 2017 | Rösmann C, Hoffmann F, Bertram T. Integrated online trajectory planning and optimization in distinctive topologies. *Robotics and Autonomous Systems* 88:142–153, 2017. doi:10.1016/j.robot.2016.11.007 | Crossref |

The five-author title *"Trajectory modification considering dynamic constraints of autonomous robots"* is the ROBOTIK 2012 conference paper (pp. 74–79); the citable journal TEB paper is the RAS 2017 article above.

## Verified entries

| # | DOI | Record (Crossref) |
|---|---|---|
| 1 | 10.1109/100.580977 | Fox D, Burgard W, Thrun S. IEEE Robotics & Automation Magazine 4(1):23–33, 1997 |
| 2 | 10.1177/027836498600500106 | Khatib O. The International Journal of Robotics Research 5(1):90–98, 1986 |
| 3 | 10.1109/70.88137 | Borenstein J, Koren Y. IEEE Transactions on Robotics and Automation 7(3):278–288, 1991 |
| 4 | 10.1177/027836499801700706 | Fiorini P, Shiller Z. The International Journal of Robotics Research 17(7):760–772, 1998 |
| 5 | 10.2514/1.G001921 | Williams G, Aldrich A, Theodorou EA. Journal of Guidance, Control, and Dynamics 40(2):344–357, 2017 |
| 6 | 10.1016/j.robot.2016.11.007 | Rösmann C, Hoffmann F, Bertram T. Robotics and Autonomous Systems 88:142–153, 2017 |
| 7 | 10.1016/S0001-4575(00)00019-1 | Minderhoud MM, Bovy PHL. Accident Analysis & Prevention 33(1):89–97, 2001 |
| 8 | 10.1109/ROBOT.1996.511023 | Simmons R. Proc. IEEE ICRA 1996;4:3375–3382 |
| 9 | 10.1109/ROBOT.1999.770002 | Brock O, Khatib O. Proc. 1999 IEEE ICRA 1999;1:341–346 |
| 10 | 10.1109/ROBOT.2007.363613 | Seder M, Petrović I. Proc. 2007 IEEE ICRA 2007:1986–1991 |
| 11 | 10.1109/ICRA.2019.8794386 | Missura M, Bennewitz M. 2019 IEEE ICRA 2019:8620–8626 |
| 12 | 10.1109/ROBOT.2008.4543489 | van den Berg J, Lin M, Manocha D. 2008 IEEE ICRA 2008:1928–1935 |
| 13 | 10.1007/978-3-642-19457-3_1 | van den Berg J, Guy SJ, Lin M, Manocha D. Robotics Research (STAR) 2011;70:3–19 |
| 14 | 10.1109/ROBOT.2009.5152817 | Ratliff N, Zucker M, Bagnell JA, Srinivasa S. 2009 IEEE ICRA 2009:489–494 |
| 15 | 10.1109/ICRA.2011.5980280 | Kalakrishnan M, Chitta S, Theodorou E, Pastor P, Schaal S. 2011 IEEE ICRA 2011:4569–4574 |
| 16 | 10.1177/0278364914528132 | Schulman J, et al. The International Journal of Robotics Research 33(9):1251–1270, 2014 |
| 17 | 10.1109/ICRA.2011.5980223 | McNaughton M, Urmson C, Dolan JM, Lee JW. 2011 IEEE ICRA 2011:4889–4895 |
| 18 | 10.1109/TIV.2016.2578706 | Paden B, Čáp M, Yong SZ, Yershov D, Frazzoli E. IEEE T-IV 1(1):33–55, 2016 |
| 19 | 10.1016/j.aap.2010.03.021 | Laureshyn A, Svensson Å, Hydén C. Accident Analysis & Prevention 42(6):1637–1646, 2010 |
| 20 | 10.1109/ICRA.2017.7989182 | Pfeiffer M, Schaeuble M, Nieto J, Siegwart R, Cadena C. 2017 IEEE ICRA 2017:1527–1533 |
| 21 | 10.1109/ICRA.2018.8460487 | Codevilla F, Müller M, López A, Koltun V, Dosovitskiy A. 2018 IEEE ICRA 2018:4693–4700 |
| 22 | 10.1109/ITSC.2018.8569938 | Lopez PA, et al. 2018 21st IEEE ITSC 2018:2575–2582 |
| 23 | 10.1109/IVS.2017.7995802 | Althoff M, Koschi M, Manzinger S. 2017 IEEE IV 2017:719–726 |

## Notes

- Cross-validation: the Fox 1997 record's own reference list independently cites DOI 10.1177/027836498600500106 (Khatib) and 10.1109/70.88137 (Borenstein & Koren).
- Reference 10 (Seder & Petrović 2007) is the closest prior work: a dynamic-window method that also uses time-to-collision for moving obstacles. It is cited in Related Work with an explicit scope distinction.
- No reference was fabricated; all 23 DOIs resolve to the cited work.
