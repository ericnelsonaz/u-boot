#include <stdio.h>
#include <stdint.h>

struct imx_header_t {
	uint32_t	sig;
	uint32_t	entry;
	uint32_t	rsvd1;
	uint32_t	dcd;
	uint32_t	bdata;
	uint32_t	self;
	uint32_t	csf;
	uint32_t	rsvd2;
};

struct boot_data_t {
	uint32_t	start;
	uint32_t	len;
	uint32_t	plugin;
};

#define ___swab32(x) \
	((uint32_t)( \
		(((uint32_t)(x) & (uint32_t)0x000000ffUL) << 24) | \
		(((uint32_t)(x) & (uint32_t)0x0000ff00UL) <<  8) | \
		(((uint32_t)(x) & (uint32_t)0x00ff0000UL) >>  8) | \
		(((uint32_t)(x) & (uint32_t)0xff000000UL) >> 24) ))

static void dump_dcd(struct imx_header_t *header,
		     struct boot_data_t *bdata,
		     char const *data)
{
	if (header->dcd) {
		uint32_t offs = (header->dcd-header->self);
		uint32_t *dcd = offs ? (uint32_t *)(data+offs) : 0;
		uint32_t dcdhead = dcd ? *dcd++ : 0;
		uint8_t tag = dcdhead & 0xff ;
		if (0xd2 == tag) {
			uint16_t dcdlen = (((dcdhead>>8)&0xFF)<<8)
					| ((dcdhead>>16)&0xFF);
			uint8_t ver = dcdhead >> 24 ;
			uint32_t cmd = *dcd++ ;
			uint8_t cmdtag = cmd&0xff ;
	
			if (0xcc == cmdtag) {
				printf("-------DCD at offset %x: 0x%08x, tag %02x, len 0x%04x, ver %02x\n",offs, dcdhead,tag,dcdlen,ver);
				printf("-------CMD 0x%08x\n", cmd);
				dcdlen -= 8 ;
				while (0 < dcdlen) {
					uint32_t addr = *dcd++ ;
					uint32_t value = *dcd++ ;
					printf("%08x\t%08x\n",___swab32(addr), ___swab32(value));
					dcdlen -= 8 ;
				}
			}
		}
	} else
		printf("no DCD\n");
}

int main( int argc, char const *const argv[])
{
	int arg;
	for( arg=1; arg<argc; arg++ ) {
		FILE *fIn;
		fIn = fopen( argv[arg], "rb");
		if (fIn) {
			printf("%s\n", argv[arg]);
                        struct imx_header_t header ;
			if (sizeof(header) == fread(&header,1,sizeof(header),fIn)) {
				if (0xd1 == (header.sig&0xff)) {
					uint32_t len = (((header.sig>>8)&0xff)<<8)
							|((header.sig>>16)&0xff);
					uint8_t ver = (header.sig>>24);
					struct boot_data_t bdata ;

					if (len > sizeof(header)) {
						fseek(fIn,len-sizeof(header),SEEK_CUR);
					}
					if (sizeof(bdata) == fread(&bdata,1,sizeof(bdata),fIn)) {
						char *data ;
						printf("sig\t0x%08x\n", header.sig);
						printf("len\t0x%04x\n", len);
						printf("ver\t0x%02x\n", ver);
						printf("entry\t0x%08x\n", header.entry);
						printf("rsvd1\t0x%08x\n", header.rsvd1);
						printf("dcd\t0x%08x\n", header.dcd);
						printf("bdata\t0x%08x\n", header.bdata);
						printf("self\t0x%08x\n", header.self);
						printf("csf\t0x%08x\n", header.csf);
						printf("rsvd2\t0x%08x\n", header.rsvd2);
						printf("start\t0x%08x\n", bdata.start);
						printf("length\t0x%08x\n", bdata.len);
						printf("plugin\t0x%08x\n", bdata.plugin);
						data = (char *)malloc(bdata.len);
						if (data) {
							int numread;
							fseek(fIn, header.dcd, SEEK_SET);
                                                        numread = fread(data,1,bdata.len,fIn);
							if (bdata.len == numread){
								dump_dcd(&header,&bdata,data);
							} else {
								perror("read dcd");
							}
						} else
							perror("malloc");
					}
					else
						perror(argv[arg]);
				} else {
					fprintf(stderr,"invalid header sig 0x%08x\n",header.sig);
					return -1;
				}
			} else
				perror(argv[arg]);
			fclose(fIn);
		} else
			perror(argv[arg]);
	}
	return 0 ;
}
