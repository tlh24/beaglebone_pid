CPP = g++

GLIBS = gtk+-3.0 
GTKFLAGS = `pkg-config --cflags $(GLIBS) `
GTKLD = `pkg-config --libs $(GLIBS) `

PACKS = camlp4,dynlink,lablgtk2,lablgtk2-gl,lablgtk2-gl.gtkgl,lwt,lwt.glib,batteries,lablgl,pcre,lablgtk2.sourceview2
OCAML := ocamlfind ocamlc -g -c -syntax camlp4o -ppopt pa_extend.cmo
OCAMLOPT := ocamlfind ocamlopt -c -syntax camlp4o -ppopt pa_extend.cmo
LINK := ocamlfind ocamlc -linkpkg -package $(PACKS) -I +camlp4 camlp4lib.cma \
	-dllpath /usr/lib/ocaml/stublibs -I /usr/lib/ocaml/stublibs
LINKOPT := ocamlfind ocamlopt -linkpkg -package $(PACKS) -I +camlp4 camlp4lib.cmxa \
	-I /usr/lib/ocaml/stublibs

CFLAGS := -g -I/usr/local/include 
CFLAGS += -Wall -Wcast-align -Wpointer-arith -Wshadow -Wsign-compare \
-Wformat=2 -Wno-format-y2k -Wmissing-braces -Wparentheses -Wtrigraphs \
-Wextra -pedantic -std=c++11 -Wno-int-to-pointer-cast 
LDFLAGS = -lieee1284 -lmatio -lhdf5_serial

OBJS = test_1284.o matStor.o sock.o

all: test1284

%.o: %.cpp 
	$(CPP) -c -o $@ $(CFLAGS) $(GTKFLAGS) $<

test1284: $(OBJS)
	$(CPP) -o $@ $(GTKLD) $(LDFLAGS) $(OBJS)
	
control.cmo: controller.ml
	$(OCAML) -package $(PACKS) -w -10-14 controller.ml -o $@
	
control: control.cmo
	$(LINK) -o $@ $<
	
control.cmx: controller.ml
	$(OCAMLOPT) -package $(PACKS) -w -10-14 controller.ml -o $@
	
control.opt: control.cmx
	$(LINKOPT) -o $@ $<
	
scene: scene.ml
	ocamlfind ocamlc -package $(PACKS) -w -10 -linkpkg scene.ml -o $@
	
browse: 
	ocamlfind ocamlbrowser -package $(PACKS)

deps:
	sudo apt-get install liblablgl-ocaml-dev liblablgtk2-gl-ocaml-dev \
	liblwt-glib-ocaml-dev liblwt-ocaml-dev libbatteries-ocaml-dev \
	liblablgtksourceview2-ocaml-dev liblablgtk2-ocaml libieee1284-3-dev \
	libboost1.55-all-dev libgtk-3-dev libmatio-dev
	
clean:
	rm -rf $(OBJS) test1284 control control.opt scene
	rm -rf *.cmi *.cmo *.cmx controller.o
