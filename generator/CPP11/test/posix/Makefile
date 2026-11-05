
CXX ?= g++
CXXFLAGS ?= -std=c++11 -g -O0 -Wall -pthread
GENDIR ?= ../../include_v2.0
GTESTSRC ?= /usr/src/gtest/src/gtest-all.cc
MSGDEFDIR ?= ../../../../../message_definitions/v1.0

DIALECTS := $(notdir $(shell find $(GENDIR) -mindepth 1 -maxdepth 1 -type d))
TARGETS := $(foreach d,$(DIALECTS),mavtest_$(d))

all: run_tests test_diagnostics test_diagnostics_positive test_gtestsuite_diagnostics test_gtestsuite_diagnostics_skipwip

gtest-all.o: $(GTESTSRC)
	$(CXX) $(CXXFLAGS) -I"$(dir $(GTESTSRC))../" -c -o $@ $<

mavtest_%: mtest.cpp gtest-all.o $(foreach d,$(DIALECTS),$(GENDIR)/$(d)/$(d).hpp)
	$(CXX) $(CXXFLAGS) -I $(GENDIR)/$* -o $@ $< gtest-all.o

run_tests: $(TARGETS)
	for f in $(TARGETS); do \
		./$$f; \
	done

# Regression test: with every diagnostic macro opted-in, the generated
# headers must not warn about their own generated code (see
# test_diagnostics.cpp). Uses -Werror regardless of the CXXFLAGS override
# above, since a silently-reintroduced misplaced attribute should fail CI.
test_diagnostics: test_diagnostics.cpp $(GENDIR)/all/all.hpp
	$(CXX) $(CXXFLAGS) -Wextra -Werror -I $(GENDIR)/all -o $@ $<
	./$@

# Positive counterpart: proves the diagnostics actually fire on real use of
# a flagged message struct or enum entry, using the dedicated
# tests/wip_deprecated_superseded.xml fixture. Expected to FAIL to build -
# see check_diagnostics_fire.sh.
# Fixture dialect used by the positive test; generated on demand so a plain
# `make` works without test_generator.sh having run first.
$(GENDIR)/wip_deprecated_superseded/wip_deprecated_superseded.hpp: ../../../../tests/wip_deprecated_superseded.xml
	../../../../tools/mavgen.py --lang C++11 -o $(GENDIR) --wire-protocol=2.0 $<

.PHONY: test_diagnostics_positive
test_diagnostics_positive: test_diagnostics_positive.cpp check_diagnostics_fire.sh $(GENDIR)/wip_deprecated_superseded/wip_deprecated_superseded.hpp
	./check_diagnostics_fire.sh $(CXX) $(CXXFLAGS) -Wextra -Werror -I $(GENDIR)/wip_deprecated_superseded

# Pins the generated gtestsuite.hpp fix history against a real dialect
# with the opt-in macros actually defined and run, in both the
# deprecated() (no skip needed) and unavailable()-plus-skip
# configurations documented in message.hpp - see the two .cpp files for
# what each one is guarding against. Deliberately no -Wextra (unlike
# test_diagnostics above): this compiles real struct field access via
# mtest.cpp against a full dialect, and -Wextra's -Wunused-parameter
# would flag mtest.cpp's own pre-existing, unrelated
# mavlink_get_msg_entry() stub. -Wno-address-of-packed-member suppresses
# the other pre-existing, unrelated noise from that same real field
# access; the per-message diagnostics this test actually cares about are
# plain -Wdeprecated-declarations/-Wattributes, covered by -Wall.
test_gtestsuite_diagnostics: test_gtestsuite_diagnostics.cpp gtest-all.o $(GENDIR)/common/common.hpp
	$(CXX) $(CXXFLAGS) -Werror -Wno-address-of-packed-member -I $(GENDIR)/common -o $@ test_gtestsuite_diagnostics.cpp gtest-all.o
	./$@

test_gtestsuite_diagnostics_skipwip: test_gtestsuite_diagnostics_skipwip.cpp gtest-all.o $(GENDIR)/common/common.hpp
	$(CXX) $(CXXFLAGS) -Werror -Wno-address-of-packed-member -I $(GENDIR)/common -o $@ test_gtestsuite_diagnostics_skipwip.cpp gtest-all.o
	./$@

clean:
	rm -f gtest-all.o $(TARGETS) test_diagnostics test_gtestsuite_diagnostics test_gtestsuite_diagnostics_skipwip

gen: $(wildcard $(MSGDEFDIR)/*.xml)
	mkdir -p $(GENDIR)
	for d in $^; do	\
		../../../../tools/mavgen.py -o $(GENDIR) --wire-protocol 2.0 --lang C $$d;	\
		../../../../tools/mavgen.py -o $(GENDIR) --wire-protocol 2.0 --lang C++11 $$d;	\
	done
