# invoke SourceDir generated makefile for httpget.pem4f
httpget.pem4f: .libraries,httpget.pem4f
.libraries,httpget.pem4f: package/cfg/httpget_pem4f.xdl
	$(MAKE) -f C:\Users\MELIKE\Desktop\4.sinif\eem449\melikeproje/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\MELIKE\Desktop\4.sinif\eem449\melikeproje/src/makefile.libs clean

