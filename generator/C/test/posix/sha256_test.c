/*
  simple test of mavlink sha256 code. 
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <mavlink_sha256.h>

// the sha256 result is just the first six bytes of a regular sha256 output,
// e.g. `printf '' | sha256sum | cut -c1-12`. then for these tables we prefix
// 0x to that result

void validate(mavlink_sha256_ctx *ctx, uint64_t expected)
{
    uint8_t result_arr[6];
    mavlink_sha256_final_48(ctx, result_arr);

    uint64_t result = 0;
    result |= ((uint64_t)result_arr[0]) << 40;
    result |= ((uint64_t)result_arr[1]) << 32;
    result |= ((uint64_t)result_arr[2]) << 24;
    result |= ((uint64_t)result_arr[3]) << 16;
    result |= ((uint64_t)result_arr[4]) << 8;
    result |= ((uint64_t)result_arr[5]) << 0;

    if (result != expected) {
        printf("FAILED: expected %012" PRIx64 ", got %012" PRIx64 "\n",
            expected, result);
        exit(1);
    }
}

void run_test(const char *input, size_t length, uint64_t expected)
{
    mavlink_sha256_ctx ctx;

    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, input, length);
    validate(&ctx, expected);
}

typedef struct {
    const char *input;
    uint64_t result; // first six bytes
} string_test_t;

static const string_test_t string_tests[] = {
    {"", 0xe3b0c44298fc},
    {"Hello, world!", 0x315f5bdb76d0},
    {"\xfd\x23\x01\x01\x01", 0xc56152f79cb7},
};

void run_string_tests(void)
{
    for (int i=0; i<(sizeof(string_tests)/sizeof(string_tests[0])); i++) {
        const string_test_t *test = &string_tests[i];

        run_test(test->input, strlen(test->input), test->result);
    }
}

typedef struct {
    char input[16];
    uint64_t result; // first six bytes
} bin_test_t;

static const bin_test_t bin_tests[] = {
    // these are padded with zeros to 16 bytes
    {"", 0x374708fff771},
    {"Hello, world!", 0xd87443b8fc88},
    {"\xfd\x23\x01\x01\x01", 0x0e40dd48bb95},
    {"26\xcb\xcb&\x12[|\xfez?\xb3\x8e\x03\xe4", 0x33ffd10eceae},
    {"\xb3;\xc3\rq\n\xd4\xab}\x97\xcev\x8f+Z", 0x3127873b8ce5},
    {"\n !+\xcd%X#\xa1\xa4^.O\x9d#\xb8", 0x87444c012c7a},
};

void run_bin_tests(void)
{
    for (int i=0; i<(sizeof(bin_tests)/sizeof(bin_tests[0])); i++) {
        const bin_test_t *test = &bin_tests[i];

        run_test(test->input, sizeof(test->input), test->result);
    }
}

void run_tests(void)
{
    run_string_tests();
    run_bin_tests();
}

int main(int argc, const char *argv[])
{
    mavlink_sha256_ctx ctx;
    uint8_t result[6];
    uint8_t i;

    if (argc == 1) {
        run_tests(); // failure causes call to exit()
        printf("Tests passed\n");
        return 0;
    }

    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, argv[1], strlen(argv[1]));
    mavlink_sha256_final_48(&ctx, result);
    for (i=0; i<6; i++) {
        printf("%02x ", (unsigned)result[i]);
    }
    printf("\n");
    return 0;
}
