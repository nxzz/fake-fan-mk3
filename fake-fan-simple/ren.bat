set PN=fake-fan-mk2
del swpcb.zip
mkdir swpcb
del /q swpcb\*
xcopy /Y gab swpcb 
cd swpcb
ren %PN%.drl %PN%.txt
ren %PN%-B.Cu.gbl %PN%.gbl
ren %PN%-B.Mask.gbs %PN%.gbs
ren %PN%-B.SilkS.gbo %PN%.gbo
ren %PN%-Edge.Cuts.gm1 %PN%.gml
ren %PN%-F.Cu.gtl %PN%.gtl
ren %PN%-F.Mask.gts %PN%.gts
ren %PN%-F.SilkS.gto %PN%.gto
7z a ..\%PN%-swpcb.zip *.*