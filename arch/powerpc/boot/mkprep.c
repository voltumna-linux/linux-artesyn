/*
 * Create a PReP boot image from a flat binary.
 *
 * Wraps a flat binary with a PReP boot partition header (1024 bytes)
 * so that MOTLoad/PPCBUG can locate the entry point via nbo.
 *
 * Based on arch/ppc/boot/utils/mkprep.c by Cort Dougan.
 *
 * Copyright 2008 Alessio Igor Bogani
 *
 * This program is licensed under the terms of the GNU General Public
 * License version 2.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* PReP partition table entry */
typedef struct {
	uint8_t		boot_indicator;
	uint8_t		starting_head;
	uint8_t		starting_sector;
	uint8_t		starting_cylinder;
	uint8_t		system_indicator;
	uint8_t		ending_head;
	uint8_t		ending_sector;
	uint8_t		ending_cylinder;
	uint8_t		beginning_sector[4];
	uint8_t		number_of_sectors[4];
} partition_entry_t;

#define BOOT_ACTIVE	0x80
#define SYSTEM_PREP	0x41
#define PREP_ENTRY	0x400	/* Code starts at offset 0x400 */

static void store_le32(uint32_t v, uint8_t *p)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff;
	p[3] = (v >> 24) & 0xff;
}

int main(int argc, char *argv[])
{
	FILE *in, *out;
	uint8_t block[512];
	partition_entry_t pe;
	long code_size;
	uint32_t total_size;
	char buf[4096];
	size_t n;

	if (argc != 3) {
		fprintf(stderr, "usage: %s <flat_binary> <prep_image>\n",
			argv[0]);
		return 1;
	}

	in = fopen(argv[1], "r");
	if (!in) {
		perror(argv[1]);
		return 1;
	}

	/* Get input size */
	fseek(in, 0, SEEK_END);
	code_size = ftell(in);
	fseek(in, 0, SEEK_SET);

	total_size = PREP_ENTRY + code_size;

	out = fopen(argv[2], "w");
	if (!out) {
		perror(argv[2]);
		fclose(in);
		return 1;
	}

	/* Build the 512-byte boot block */
	memset(block, 0, sizeof(block));

	/* Entry point offset (LE32) at byte 0 */
	store_le32(PREP_ENTRY, &block[0]);

	/* Image length (LE32) at byte 4 */
	store_le32(total_size, &block[4]);

	/* MBR signature */
	block[510] = 0x55;
	block[511] = 0xAA;

	/* PReP partition table entry at offset 0x1BE */
	memset(&pe, 0, sizeof(pe));
	pe.boot_indicator = BOOT_ACTIVE;
	pe.system_indicator = SYSTEM_PREP;
	pe.starting_head = 0;
	pe.starting_sector = 2;
	pe.starting_cylinder = 0;
	pe.ending_head = 1;
	pe.ending_sector = 18;
	pe.ending_cylinder = 79;
	store_le32(0, pe.beginning_sector);
	store_le32(2 * 18 * 80 - 1, pe.number_of_sectors);
	memcpy(&block[0x1BE], &pe, sizeof(pe));

	/* Write boot block */
	fwrite(block, sizeof(block), 1, out);

	/* Write entry + length copy at offset 0x200 */
	fwrite(&block[0], 4, 1, out);	/* entry */
	fwrite(&block[4], 4, 1, out);	/* length */

	/* Seek to code area at offset 0x400 */
	fseek(out, PREP_ENTRY, SEEK_SET);

	/* Copy flat binary */
	while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
		fwrite(buf, 1, n, out);

	fclose(in);
	fclose(out);
	return 0;
}
