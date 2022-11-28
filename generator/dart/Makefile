.PHONY: test coverage

test:
	@dart test --coverage=./coverage -p "chrome,vm" test/mavlink_test.dart

NON_GENERATED_SOURCES := $(shell find lib/src/messages -name '*.dart')
NON_GENERATED_SOURCES += $(shell find lib/src -maxdepth 1 -name '*.dart')

SOURCES_TO_REPORT := $(addprefix --report-on=,$(NON_GENERATED_SOURCES))

coverage: test
	dart pub global run coverage:format_coverage -i coverage/ $(SOURCES_TO_REPORT) -l > coverage/test/mavlink_test.dart.lcov
	@perl /c/ProgramData/chocolatey/lib/lcov/tools/bin/genhtml -o ./coverage/report coverage/test/mavlink_test.dart.lcov
	@echo "Coverage report generated at $(PWD)/coverage/report/index.html"
