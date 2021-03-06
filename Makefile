# Language for word list selection.
# Can be overridden on the command line e.g. "make WORD_LIST_LANGUAGE=fr ..."
WORD_LIST_LANGUAGE := en
#WORD_LIST_LANGUAGE := fr

# eSpeak voice to use.
# Can be overriden on the command line e.g. "make VOICE=en-us ..."
VOICE ?= $(WORD_LIST_LANGUAGE)

all: writer dmm

build/words-flash-writer build/dmm-talking-arduino-sketch:
	mkdir -p $@ && cd $@ && ano init

build/Adafruit_TinyFlash:
	mkdir -p build && cd build && \
	git clone https://github.com/adafruit/Adafruit_TinyFlash

build/%/lib/Adafruit_TinyFlash \
 : patches/tiny-flash.diff | build/Adafruit_TinyFlash build/%
	rm -rf $@ && cp -a build/Adafruit_TinyFlash $@ && \
	patch -d $@ -p1 < patches/tiny-flash.diff

build/words-flash-writer/%: words-flash-writer/% | build/words-flash-writer
	cp $< $@

# There is a bug in the ano build system where updated .h files
# are not considered as a proper dependency for a rebuild.
# Let's force it by removing affected product files.
define fixdeps
	if [ "$(suffix $@)" = ".h" ]; then cd $(dir $@).. && ano clean; fi
endef

build/words-flash-writer/src/%: words-flash-writer/flash-writer-arduino-sketch/% | build/words-flash-writer
	@cp -v $< $@ && $(fixdeps)
build/dmm-talking-arduino-sketch/src/%: dmm-talking-arduino-sketch/% | build/dmm-talking-arduino-sketch
	@cp -v $< $@ && $(fixdeps)
build/dmm-talking-arduino-sketch/src/words_def.h: build/words-flash-writer/words_def.h | build/dmm-talking-arduino-sketch
	@cp -v $< $@ && $(fixdeps)

writer_src_files := \
	$(wildcard words-flash-writer/flash-writer-arduino-sketch/*)
writer_deps := \
	build/words-flash-writer/lib/Adafruit_TinyFlash \
	$(patsubst words-flash-writer/flash-writer-arduino-sketch/%, \
	           build/words-flash-writer/src/%, \
	           $(writer_src_files))

dmm_src_files := \
	$(wildcard dmm-talking-arduino-sketch/*)
dmm_deps := \
	build/dmm-talking-arduino-sketch/lib/Adafruit_TinyFlash \
	build/dmm-talking-arduino-sketch/src/words_def.h \
	$(patsubst dmm-talking-arduino-sketch/%, \
	           build/dmm-talking-arduino-sketch/src/%, \
	           $(dmm_src_files))

build/words-flash-writer/snd.data \
build/words-flash-writer/words_def.h \
 : build/words-flash-writer/make.py build/words-flash-writer/list-$(WORD_LIST_LANGUAGE)
	cd build/words-flash-writer && ./make.py --lang="$(WORD_LIST_LANGUAGE)" --voice="$(VOICE)"

writer: $(writer_deps)
	cd build/words-flash-writer && \
	ano build -m pro --cpu 8MHzatmega328

dmm: $(dmm_deps)
	cd build/dmm-talking-arduino-sketch && \
	ano build -m pro --cpu 8MHzatmega328

upload_dmm: dmm
	cd build/dmm-talking-arduino-sketch && \
	ano upload -m pro --cpu 8MHzatmega328

upload_words: build/words-flash-writer/snd.data build/words-flash-writer/sender.py writer
	cd build/words-flash-writer && \
	ano upload -m pro --cpu 8MHzatmega328 && \
	sleep 2 && \
	./sender.py

everything: upload_words upload_dmm

watch:
	ano serial -b 115200

asmdump:
	avr-objdump -d -S build/dmm-talking-arduino-sketch/.build_ano/pro/firmware.elf | less

clean:
	rm -rf build/words-flash-writer/
	rm -rf build/dmm-talking-arduino-sketch/

distclean:
	rm -rf build/

.PHONY: writer dmm upload_dmm upload_words everything watch asmdump clean distclean

GNUMAKEFLAGS := --no-print-directory
