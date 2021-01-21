#include "lm_bbdinf.hpp"

/*
 function: crc checking and deconverging without 8to32
  idata[i]
  finf[o]
  odata[o]
 */
void lm_bbdchk(stream<U8> &idata, stream<lm_frm_info> &finf, stream<U8> &odata)
{
#pragma HLS pipeline II=1 enable_flush

	static enum enState {RS_NIL = 0, RS_1TYPE, RS_AFT1TYPE, RS_TYPE, RS_AFTTYPE, RS_LEN1, RS_LEN2, RS_RXING, RS_TRXING, RS_PADING, RS_CRC1, RS_CRC2} rfs;
	static CRC_VAL cval, ccval;
	static ap_uint<12> rbc, fstp, fedp;

	static ap_uint<3> frms;
	static ap_uint<7> ftyp;
	static ap_uint<1> nh;
	static ap_uint<8> send;
	static ap_uint<3> cnt;

	static ap_uint<12> flen;
	static lm_frm_info info;
#pragma HLS ARRAY_PARTITION variable=info.flen complete dim=1
#pragma HLS ARRAY_PARTITION variable=info.fpos complete dim=1
#pragma HLS ARRAY_PARTITION variable=info.ftyp complete dim=1

	static U32 tick;
	static U32 fttag;

	switch(rfs) {
		case RS_NIL:

			fedp = 0;
			cval = 0;
			ccval = 0;
			frms = 0;
			fstp = 0;
			cnt = 0;

			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				lm_calcrc_8toV(val8, ccval);
				odata.write(val8);
				info.send = val8;

				rfs = RS_1TYPE;
				rbc = 1;

				/* record time tag */
				fttag = tick;

				lm_clrxinfo(info);
			}
			break;

		case RS_1TYPE:
			if (false == idata.empty()) {
				info.ttag = fttag;
				
				U8 val8, tmp;
				idata.read(val8);

				lm_calcrc_8toV(val8, ccval);

				tmp = val8(6, 0);
				odata.write(tmp);

				ftyp = val8(6, 0);
				nh = val8(7,7);

				lm_ftp2len(ftyp, flen);

				rfs = RS_AFT1TYPE;
			}
			break;

		case RS_AFT1TYPE: {
			if (flen > 0) {
				info.ftyp[frms] = ftyp;
				info.fpos[frms] = 0;
				info.flen[frms] = flen + 1;

				fstp = 0;
				fedp = flen;

				if (1 == nh) {
					rfs = RS_RXING;
				}
				else {
					rfs = RS_TRXING;
				}
			}
			else {
				info.ftyp[frms] = ftyp;
				info.fpos[frms] = 0;

				fstp = 0;
				flen = 2;
				cnt = 1;

				rfs = RS_LEN1;
			}

			rbc += 1;
			break;
		}

		case RS_TYPE:
			if (false == idata.empty()) {
				U8 val8, tmp;
				idata.read(val8);
				lm_calcrc_8toV(val8, ccval);

				tmp = val8(6, 0);
				odata.write(tmp);

				ftyp = val8(6, 0);
				nh = val8(7,7);
				lm_ftp2len(ftyp, flen);

				rfs = RS_AFTTYPE;
			}
			break;

		case RS_AFTTYPE: {
			if (flen > 0) {
				info.ftyp[frms] = ftyp;
				info.fpos[frms] = rbc;
				info.flen[frms] = flen;

				fstp = rbc;
				fedp = rbc + flen - 1;

				if (1 == nh) {
					rfs = RS_RXING;
				}
				else {
					rfs = RS_TRXING;
				}
			}
			else {
				info.ftyp[frms] = ftyp;
				info.fpos[frms] = rbc;

				fstp = rbc;
				flen = 1;
				cnt = 1;

				rfs = RS_LEN1;
			}

			rbc += 1;
			break;
		}

		case RS_LEN1:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				lm_calcrc_8toV(val8, ccval);
				odata.write(val8);

				if (cnt == 7) {
					/* 1-byte length */
					if (0 == val8(7, 7)) {
						info.flen[frms] = flen + val8(6, 0);
						fedp = fstp + flen + val8(6, 0) - 1;

						if (1 == nh) {
							rfs = RS_RXING;
						}
						else {
							rfs = RS_TRXING;
						}
					}
					/* 2-byte length */
					else {
						ap_uint<12> tmp;
						tmp = val8(3,0);
						flen += tmp<<8;
						rfs = RS_LEN2;
					}
				}
				else {
					flen += 1;
					cnt += 1;
				}

				rbc += 1;
			}
			break;

		case RS_LEN2:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				lm_calcrc_8toV(val8, ccval);
				odata.write(val8);

				flen += val8;

				info.flen[frms] = flen;
				fedp = fstp + flen - 1;

				if (1 == nh) {
					rfs = RS_RXING;
				}
				else {
					rfs = RS_TRXING;
				}

				rbc += 1;
			}
			break;

		case RS_RXING:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				lm_calcrc_8toV(val8, ccval);
				odata.write(val8);

				if (rbc == fedp) {
					frms += 1;
					info.cfrm += 1;

					rfs = RS_TYPE;
				}

				rbc += 1;
			}
			break;

		case RS_TRXING:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				lm_calcrc_8toV(val8, ccval);
				odata.write(val8);

				if (fedp == rbc) {
					frms += 1;
					info.cfrm += 1;
					info.tlen = rbc + 1;

					if (rbc + 1 == MR_BB_MTU - MR_CRC_LEN) {
						rfs = RS_CRC1;
					}
					else {
						rfs = RS_PADING;
					}
				}

				rbc += 1;
			}
			break;

		case RS_PADING:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				rbc += 1;

				if (rbc == MR_BB_MTU - MR_CRC_LEN) {
					rfs = RS_CRC1;
				}
			}
			break;

		case RS_CRC1:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				rbc += 1;

				cval(7, 0) = val8;
				rfs = RS_CRC2;
			}
			break;

		case RS_CRC2:
			if (false == idata.empty()) {
				U8 val8;
				idata.read(val8);
				rbc += 1;

				cval(15, 8) = val8;

				if (cval == ccval) {
					info.gfrm = 1;
					finf.write(info);
#ifndef __SYNTHESIS__
					short fcnt = info.cfrm;
					short alen = info.tlen;
					UW16 crc = cval;
					printf("[crc]: a good cvg frame, count=%d, length=%d, crc=%x\n", fcnt, alen, crc);
#endif
				}
				else {
					info.gfrm = 0;
					finf.write(info);
#ifndef __SYNTHESIS__
					unsigned short crc1, crc2;
					crc1 = cval;
					crc2 = ccval;
					printf("[crc]: drop a bad frame due to incorrect crc, ccval=%x, cval=%x\n", crc2, crc1);
#endif
				}

				rfs = RS_NIL;
			}
			break;
	}

	tick = tick + 1;
//	UW64 tmp = tick;
//	printf("tick_bb=%d\n",tmp);
}

/*
 function: bb data input, 8to32 and dispatch to different module
 */
void lm_bbdproc(stream<lm_frm_info> &finf, stream<U8> &rfrm, stream<lm_idu_dwd> &dfrm, stream<lm_idu_dwd> &sfrm, stream<lm_idu_dwd> &cfrm)
{
#pragma HLS pipeline II=1 enable_flush

	static ap_uint<12> rbc, flen;
	static U8 type;
	static ap_uint<2> bpos;
	static ap_uint<3> fidx;

	static lm_idu_dwd cword;
	static lm_frm_info inf;
#pragma HLS ARRAY_PARTITION variable=inf.flen complete dim=1
#pragma HLS ARRAY_PARTITION variable=inf.fpos complete dim=1
#pragma HLS ARRAY_PARTITION variable=inf.ftyp complete dim=1

	static enum enState {BPS_NIL = 0, BPS_DISP, BPS_DFWDING, BPS_SFWDING, BPS_CFWDING, BPS_NEWFRM, BPS_DROPPING} bbps;

	switch(bbps) {
		case BPS_NIL: {
			if (false == finf.empty()) {
				if (false == rfrm.empty()) {


					fidx = 0;
					rbc = 0;

					finf.read(inf);

					/* forward */
					if (1 == inf.gfrm) {
#ifndef __SYNTHESIS__
						unsigned short num[4];
						num[0] = inf.ftyp[fidx];
						num[1] = inf.fpos[fidx];
						num[2] = inf.flen[fidx];
						num[3] = rbc;
						EPT2("[fd]: new frame, type=%d, pos=%d, len=%d, rbc=%d\n", num[0], num[1], num[2], num[3]);
#endif

						type = inf.ftyp[fidx];

						cword.data(15, 0) = 4 + inf.flen[fidx];
						cword.data(19, 16) = 1; // rx frames
						cword.data(31, 20) = inf.ttag(11, 0); // time tag
						cword.last = 0;

						bpos = 0;
						bbps = BPS_DISP;
					}
					/* drop */
					else {
						U8 val8;
						rfrm.read(val8);
						rbc += 1;

						bbps = BPS_DROPPING;
					}
				}
			}
			break;
		}

		case BPS_DISP:
			if (type < PKT2_COD_BASE) {
				if (type == PKT_LNK_SERFB || type == PKT_LNK_SERFS) {
					cfrm.write(cword);
					sfrm.write(cword);
					bbps = BPS_SFWDING;
				}
				else {
					cfrm.write(cword);
					bbps = BPS_CFWDING;
				}
			}
			else if (type < PKT2_LBL_BASE) {
				EPT_HERE;
				dfrm.write(cword);
				bbps = BPS_DFWDING;
			}
			else {
				U8 val8;
				rfrm.read(val8);
				rbc += 1;

				bbps = BPS_DROPPING;
			}
			break;

		case BPS_DROPPING: {
			if (false == rfrm.empty()) {
				U8 val8;
				rfrm.read(val8);
				rbc += 1;

				if (rbc == inf.tlen) {
					bbps = BPS_NIL;
				}
			}
			break;
		}

		case BPS_DFWDING: {
			if (false == rfrm.empty()) {
				U8 val8;
				rfrm.read(val8);
				rbc += 1;
				bpos += 1;
				flen += 1;

				lm_byte2dword(bpos, val8, cword.data);

				if (flen == inf.flen[fidx]) {
					cword.last = 1;
					dfrm.write(cword);

					if (rbc != inf.tlen) {
						fidx += 1;
						bbps = BPS_NEWFRM;
					}
					else {
						bbps = BPS_NIL;
					}
				}
				else if (bpos == 0) {
					dfrm.write(cword);
				}
			}
			break;
		}

		case BPS_SFWDING: {
			if (false == rfrm.empty()) {
				U8 val8;
				rfrm.read(val8);
				rbc += 1;
				bpos += 1;
				flen += 1;

				lm_byte2dword(bpos, val8, cword.data);

				if (flen == inf.flen[fidx]) {
#ifndef __SYNTHESIS__
					EPT2("[fd]: end of dispatching a new sfm.\n");
#endif
					cword.last = 1;
					cfrm.write(cword);
					sfrm.write(cword);

					if (rbc != inf.tlen) {
						fidx += 1;
						bbps = BPS_NEWFRM;
					}
					else {
						bbps = BPS_NIL;
					}
				}
				else if (bpos == 0) {
					cfrm.write(cword);
					sfrm.write(cword);
				}
			}
			break;
		}

		case BPS_CFWDING: {
			if (false == rfrm.empty()) {
				U8 val8;
				rfrm.read(val8);
				rbc += 1;
				bpos += 1;
				flen += 1;

				lm_byte2dword(bpos, val8, cword.data);

				if (flen == inf.flen[fidx]) {
					cword.last = 1;
					cfrm.write(cword);

					if (rbc != inf.tlen) {
						fidx += 1;
						bbps = BPS_NEWFRM;
					}
					else {
						bbps = BPS_NIL;
					}
				}
				else if (bpos == 0) {
					cfrm.write(cword);
				}
			}
			break;
		}

		case BPS_NEWFRM: {
			lm_idu_dwd tword;

#ifndef __SYNTHESIS__
			unsigned short num[4];
			num[0] = inf.ftyp[fidx];
			num[1] = inf.fpos[fidx];
			num[2] = inf.flen[fidx];
			num[3] = rbc;
			EPT2("[fd]: new frame, type=%d, pos=%d, len=%d, rbc=%d\n", num[0], num[1], num[2], num[3]);
#endif

			/* add send node */
			type = inf.ftyp[fidx];
			flen = 0;
			bpos = 1;

			cword.data(7, 0) = inf.send;
			cword.last = 0;

			/* add frame length, send node */
			tword.data(15, 0) = 4 + inf.flen[fidx] + 1;
			tword.data(19, 16) = 1; // rx frame
			tword.data(31, 20) = inf.ttag(11, 0); // time tag
			tword.last = 0;

			if (type < PKT2_COD_BASE) {
				if (type == PKT_LNK_SERFB || type == PKT_LNK_SERFS) {
					cfrm.write(tword);
					sfrm.write(tword);
					bbps = BPS_SFWDING;

				}
				else {
					cfrm.write(tword);
					bbps = BPS_CFWDING;

				}
			}
			else if (type < PKT2_LBL_BASE) {
				dfrm.write(tword);
				bbps = BPS_DFWDING;
			}
			else {
				U8 val8;
				rfrm.read(val8);
				rbc += 1;

				bbps = BPS_DROPPING;
			}
		}
	}

}

/*
 function: frame filter
  ifrm[i]
  icmd[i]
  vld[o]
  ofrm[o]
 */
void lm_frmfilter(stream<lm_idu_dwd> &icmd, stream<lm_idu_dwd> &ifrm, stream<T_SIG_EN> &vld, stream<lm_idu_dwd> &ofrm)
{
#pragma HLS pipeline II=1 enable_flush

	static enum enState {RC_NIL = 0, RC_IDLE, RC_SASET, RC_LOOP} rcs;
	static ap_uint<2> sflg;
	static ap_uint<12> cfl;
	static U8 sa;
	static U8 sa_bk;

	static enum enFFState {FF_NIL = 0, FF_DROP, FF_TYPE, FF_ACMP, FF_ING} ffs;
	static ap_uint<12> dlen;

	switch(rcs) {
		case RC_NIL:
			if (false == icmd.empty()) {
				lm_idu_dwd value;
				icmd.read(value);

				cfl = value.data(11, 0); 

				rcs = RC_SASET;
			}
			break;

		case RC_IDLE:
			if (false == icmd.empty()) {
				lm_idu_dwd value;
				icmd.read(value);

				cfl = value.data(11, 0); 

				rcs = RC_SASET;
			}
			break;
			
		case RC_SASET:
			if (false == icmd.empty()) {
				lm_idu_dwd value;
				icmd.read(value);

				if (sflg == 0) {
					sa = value.data(7, 0);
					sflg = 1;
				}
				else {
					sa_bk = value.data(7, 0);
					sflg = 2;
				}
				
				if (value.last == 1) {
					rcs = RC_IDLE;
				}
				else {
					rcs = RC_LOOP;
				}
			}			
			break;

		case RC_LOOP:
			if (false == icmd.empty()) {
				lm_idu_dwd value;
				icmd.read(value);
				
				if (value.last == 1) {
					rcs = RC_IDLE;
				}
			}
			break;			
	}

	switch(ffs) {
		case FF_NIL:
			if (false == ifrm.empty()) {
				lm_idu_dwd value;
				ifrm.read(value);

				dlen = value.data(11, 0);

				ofrm.write(value);
				ffs = FF_TYPE;
			}
			else if (sflg == 2) {
				sa = sa_bk;
				sflg = 1;
			}
			break;

		case FF_TYPE:
			if (false == ifrm.empty()) {
				lm_idu_dwd value;
				ifrm.read(value);

				ofrm.write(value);
				ffs = FF_ACMP;
			}
			else if (sflg == 2) {
				sa = sa_bk;
				sflg = 1;
			}
			break;

		case FF_ACMP:
			if (false == ifrm.empty()) {
				if (sflg == 0) {
					lm_idu_dwd value;
					ifrm.read(value);

					value.last = 1;
					ofrm.write(value);
					vld.write(0);

					ffs = FF_DROP;
				}
				else {
					lm_idu_dwd value;
					ifrm.read(value);

					U8 recv;
					recv = value.data(7, 0);

					if (true == lm_adfilter(sa, recv)) {
						ofrm.write(value);
						vld.write(1);
						ffs = FF_ING;						
					}
					else {
						value.last = 1;
						ofrm.write(value);
						vld.write(0);
						ffs = FF_DROP;
					}
				}
			}
			break;

		case FF_ING:
			if (false == ifrm.empty()) {
				lm_idu_dwd value;
				ifrm.read(value);

				ofrm.write(value);
				if (value.last == 1) {
					ffs = FF_NIL;
				}
			}
			break;

		case FF_DROP:
			if (false == ifrm.empty()) {
				lm_idu_dwd value;
				ifrm.read(value);

				if (value.last == 1) {
					ffs = FF_NIL;
				}
			}
			else if (sflg == 2) {
				sa = sa_bk;
				sflg = 1;
			}			
			break;
	}
}

bool lm_adfilter(U8 sa, U8 recv)
{
	bool rval = false;

	if (recv == sa || recv == MR_BCAST_ADD) {
		rval = true;
	}

	return rval;
}

void lm_calcrc_8toV(U8 bval, CRC_VAL &crc)
{
	static const unsigned short Crc16Table[256] =
			{
			    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
			    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
			    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
			    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
			    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
			    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
			    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
			    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
			    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
			    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
			    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
			    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
			    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
			    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
			    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
			    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
			    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
			    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
			    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
			    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
			    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
			    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
			    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
			    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
			    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
			    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
			    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
			    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
			    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
			    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
			    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
			    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
			};
#pragma HLS ARRAY_PARTITION variable=Crc16Table complete dim=1
				    //开始计算CRC16校验值
				    unsigned short TableNo=0;

						TableNo=((crc & 0xff)^(bval & 0xff));
						crc=((crc>>8)&0xff)^Crc16Table[TableNo];
}

void lm_calcrc_32toV(ap_uint<3> cnt, U32 bval, CRC_VAL &crc)
{
	crc = 0x5a5a;
}

#if 1
void lm_ftp2len(U8 ftype, ap_uint<12> &len)
{
#pragma HLS INLINE
	const static ap_uint<8> lkpl[] = {
		31, 17, 1, 1, 2, 6, 0, 0, 5
	};
#pragma HLS ARRAY_PARTITION variable=lkpl complete dim=1

	if (ftype < PKT2_COD_BASE) {
		len = lkpl[ftype];
	}
	else {
		len = 0;
	}
}
#else
void lm_ftp2len(U8 ftype, ap_uint<12> &len)
{
#pragma HLS INLINE
/*	const static ap_uint<8> lkpl[] = {
		25, 17, 1, 1, 2, 6, 0, 0, 5
	};
*/
	switch(ftype) {
		case 0:
			len = 25;
			break;

		case 1:
			len = 17;
			break;

		case 2:
			len = 1;
			break;

		case 3:
			len = 1;
			break;

		case 4:
			len = 2;
			break;

		case 5:
			len = 6;
			break;

		case 6:
			len = 0;
			break;

		case 7:
			len = 0;
			break;

		case 8:
			len = 5;
			break;

		default:
			len = 0;
			break;
	}
}
#endif

void lm_byte2dword(ap_uint<2> pos, U8 val8, U32 &val32)
{
#pragma HLS INLINE
	if (pos == 1) {
		val32(7, 0) = val8;
		UW8 tmp = val32(7, 0);
//		printf("cword=%x\n",tmp);
	}
	else if (pos == 2) {
		val32(15, 8) = val8;
		UW8 tmp = val32(15, 8);
//		printf("cword=%x\n",tmp);
	}
	else if (pos == 3) {
		val32(23, 16) = val8;
		UW8 tmp = val32(23, 16);
//		printf("cword=%x\n",tmp);
	}
	else {
		val32(31, 24) = val8;
		UW8 tmp = val32(31, 24);
//		printf("cword=%x\n",tmp);
	}
}

void lm_clrxinfo(lm_frm_info &rinf)
{
	rinf.cfrm = 0;
	rinf.gfrm = 0;
	rinf.tlen = 0;

	for (int i = 0; i < LM_CFRM_CNT; ++i) {
		rinf.flen[i] = 0;
		rinf.fpos[i] = 0;
		rinf.ftyp[i] = 0;
	}
}

void lm_btxdata(stream<lm_bbt_dwd> &idata, stream<U8> &odata)
{
#pragma HLS pipeline II=1 enable_flush

	static enum enState {BTS_NIL = 0, BTS_BYTE0, BTS_BYTE1, BTS_BYTE2, BTS_BYTE3, BTS_FEND, BTS_PADDING, BTS_CRC1, BTS_CRC2} bts;
	
	static ap_uint<10> plen;
	static lm_bbt_dwd tword;
	static U8 bval;
	static U16 cval;

	switch(bts) {
		case BTS_NIL:
			if (false == idata.empty()) {
				idata.read(tword);

				bval = tword.data(7, 0);
				odata.write(bval);
				lm_calcrc_8toV(bval, cval);

				cval = 0;
				plen = 1;
				bts = BTS_BYTE1;
			}
			break;

		case BTS_BYTE0:{
			idata.read(tword);

			bval = tword.data(7, 0);
			odata.write(bval);
			lm_calcrc_8toV(bval, cval);

			plen = plen + 1;

			if (3 == tword.ivbs) {
				bts = BTS_FEND;
			}
			else {
				bts = BTS_BYTE1;
			}
			break;
		}

		case BTS_BYTE1: {
			bval = tword.data(15, 8);
			odata.write(bval);
			lm_calcrc_8toV(bval, cval);
			
			plen = plen + 1;

			if (2 == tword.ivbs) {
				bts = BTS_FEND;
			}
			else {
				bts = BTS_BYTE2;
			}

			break;
		}

		case BTS_BYTE2: {
			bval = tword.data(23, 16);
			odata.write(bval);
			lm_calcrc_8toV(bval, cval);
			
			plen = plen + 1;

			if (1 == tword.ivbs) {
				bts = BTS_FEND;
			}
			else {
				bts = BTS_BYTE3;
			}

			break;
		}

		case BTS_BYTE3: {
			bval = tword.data(31, 24);
			odata.write(bval);
			lm_calcrc_8toV(bval, cval);
			
			plen = plen + 1;

			if (1 == tword.last) {
				bts = BTS_FEND;
			}
			else {
				bts = BTS_BYTE0;
			}

			break;
		}

		case BTS_FEND:{
			if (plen == MR_LMIDU_PL) {
				bval = cval(7, 0);
				odata.write(bval);
				bts = BTS_CRC2;
			}
			else if (plen + 1 == MR_LMIDU_PL) {
				odata.write(0);
				lm_calcrc_8toV(0, cval);
				bts = BTS_CRC1;
			}
			else if (false == idata.empty()) {
				idata.read(tword);

				bval = tword.data(7, 0);
				odata.write(bval);
				lm_calcrc_8toV(bval, cval);

				plen = plen + 1;
				bts = BTS_BYTE1;
			}
			else {					
				odata.write(0);
				lm_calcrc_8toV(0, cval);
				plen = plen + 1;
				bts = BTS_PADDING;
			}

			break;
		}

		case BTS_PADDING: {
			odata.write(0);
			lm_calcrc_8toV(0, cval);

			if (plen + 1 == MR_LMIDU_PL) {
				bts = BTS_CRC1;				
			}
			else {
				plen = plen + 1;
			}
			break;
		}

		case BTS_CRC1: {
			bval = cval(7, 0);
			odata.write(bval);
			bts = BTS_CRC2;
			break;
		}

		case BTS_CRC2: {
			bval = cval(15, 8);
			odata.write(bval);
#ifndef __SYNTHESIS__
			printf("*****BB OUT SUCCESS!!!*****\n");
#endif
			bts = BTS_NIL;
			break;
		}		
		
	}

}
