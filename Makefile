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

build/dmm-talking-arduino-sketch/src/SoftwareSerial.cpp \
 : patches/SoftwareSerial.diff | build/dmm-talking-arduino-sketch
	# Get SoftwareSerial.cpp from arduino development tools.
	# It's either directly there or under src/.
	rm -f $@
	find /usr/share/arduino/hardware/arduino/avr/libraries/SoftwareSerial \
	    -name SoftwareSerial.cpp -exec cp -v "{}" $@ \;
	patch $@ < patches/SoftwareSerial.diff

build/words-flash-writer/%: words-flash-writer/% | build/words-flash-writer
	cp $< $@

# There is a bug in the ano build system where updated .h files
# are not considered as a proper dependency for a rebuild.
# Let's force it by removing affected product files.
define fixdeps
	if [ "$(suffix $@)" = ".h" ]; \
	then for F in $$(cd $(dir $@) && grep -l $(notdir $@) *); \
	do rm -vf "$(dir $@)../.build_ano/pro/src/$$F"*; \
	done; fi
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
 : build/words-flash-writer/make.py build/words-flash-writer/list
	cd build/words-flash-writer && ./make.py

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
	cd build/dmm-talking-arduino-sketch && \
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
