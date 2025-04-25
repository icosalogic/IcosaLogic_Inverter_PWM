/*
 * Define the TCC IOSETs used in the SAMD51 chips.
 * See section 6.2.8.5 in the SAMD51 datasheet 2021.
 */

#ifndef _I20_TCC_IOSET_
#define _I20_TCC_IOSET_

typedef struct {
  char       pad[8];
} TccPad;

typedef struct {
  uint8_t    tccNum;
  uint8_t    iosetNum;
  uint8_t    numCh;
  uint8_t    numPads;
  TccPad     pads[8];
} TccIoset;

typedef struct {
  uint8_t    numIosets;
  uint8_t    counterBits;
  bool       hasDti;
  TccIoset*  ioset[6];
} TccIosetList;

TccIoset tcc0Ioset1 = {0, 1, 6, 8, {{"PA08"}, {"PA09"}, {"PA10"}, {"PA11"}, {"PB10"}, {"PB11"}, {"PA12"}, {"PA13"}}};
TccIoset tcc0Ioset2 = {0, 2, 6, 8, {{"PC04"}, {"PD08"}, {"PD09"}, {"PD10"}, {"PD11"}, {"PD12"}, {"PC22"}, {"PC23"}}};
TccIoset tcc0Ioset3 = {0, 3, 6, 8, {{"PC10"}, {"PC11"}, {"PC12"}, {"PC13"}, {"PC14"}, {"PC15"}, {"PA18"}, {"PA19"}}};
TccIoset tcc0Ioset4 = {0, 4, 6, 8, {{"PC16"}, {"PC17"}, {"PC18"}, {"PC19"}, {"PC20"}, {"PC21"}, {"PB30"}, {"PB31"}}};
TccIoset tcc0Ioset5 = {0, 5, 6, 6, {{"PB12"}, {"PB13"}, {"PB14"}, {"PB15"}, {"PA16"}, {"PA17"}}};
TccIoset tcc0Ioset6 = {0, 6, 6, 6, {{"PA20"}, {"PA21"}, {"PA22"}, {"PA23"}, {"PB16"}, {"PB17"}}};

TccIoset tcc1Ioset1 = {1, 1, 4, 8, {{"PA16"}, {"PA17"}, {"PA18"}, {"PA19"}, {"PA20"}, {"PA21"}, {"PA22"}, {"PA23"}}};
TccIoset tcc1Ioset2 = {1, 2, 4, 8, {{"PD20"}, {"PD21"}, {"PB20"}, {"PB21"}, {"PB28"}, {"PB29"}, {"PA10"}, {"PA11"}}};
TccIoset tcc1Ioset3 = {1, 3, 4, 8, {{"PB18"}, {"PB19"}, {"PB26"}, {"PB27"}, {"PA08"}, {"PA09"}, {"PC12"}, {"PC13"}}};
TccIoset tcc1Ioset4 = {1, 4, 4, 6, {{"PB10"}, {"PB11"}, {"PA12"}, {"PA13"}, {"PC10"}, {"PC11"}}};
TccIoset tcc1Ioset5 = {1, 5, 4, 4, {{"PC14"}, {"PC15"}, {"PA14"}, {"PA15"}}};

TccIoset tcc2Ioset1 = {2, 1, 3, 3, {{"PA14"}, {"PA15"}, {"PA24"}}};
TccIoset tcc2Ioset2 = {2, 2, 3, 3, {{"PA30"}, {"PA31"}, {"PB02"}}};

TccIoset tcc3Ioset1 = {3, 1, 2, 2, {{"PB12"}, {"PB13"}}};
TccIoset tcc3Ioset2 = {3, 2, 2, 2, {{"PB16"}, {"PB17"}}};

TccIoset tcc4Ioset1 = {4, 1, 2, 2, {{"PB14"}, {"PB15"}}};
TccIoset tcc4Ioset2 = {4, 2, 2, 2, {{"PB30"}, {"PB31"}}};

TccIosetList tcc0Iosets = {6, 24, true,  {&tcc0Ioset1, &tcc0Ioset2, &tcc0Ioset3, &tcc0Ioset4, &tcc0Ioset5, &tcc0Ioset6}};
TccIosetList tcc1Iosets = {5, 24, true,  {&tcc1Ioset1, &tcc1Ioset2, &tcc1Ioset3, &tcc1Ioset4, &tcc1Ioset5}};
TccIosetList tcc2Iosets = {2, 16, false, {&tcc2Ioset1, &tcc2Ioset2}};
TccIosetList tcc3Iosets = {2, 16, false, {&tcc3Ioset1, &tcc3Ioset2}};
TccIosetList tcc4Iosets = {2, 16, false, {&tcc4Ioset1, &tcc4Ioset2}};

TccIosetList* tccIosets[5] = {&tcc0Iosets, &tcc1Iosets, &tcc2Iosets, &tcc3Iosets, &tcc4Iosets};

/*
 * Dumps all the TCC IOSETs available on this processor family.
 * Note that this is a superset of what is actually available on the board that is running.
 * See platforms/*.h for that configuration info.
 */
void dumpTccIosets() {
  for (int tccNum = 0; tccNum < TCC_INST_NUM; tccNum++) {
    TccIosetList* iosetList = tccIosets[tccNum];
    for (int iosetNum = 1; iosetNum < iosetList->numIosets; iosetNum++) {
      TccIoset* ioset = iosetList->ioset[iosetNum];
      Serial.printf("TCC%d  IOSET%d  ", ioset->tccNum, ioset->iosetNum);
      for (int i = 0; i < 8; i++) {
        if (ioset->pads[i].pad[0] == 'P') {
          Serial.printf("%4s  ", ioset->pads[i].pad);
        }
      }
    }
    Serial.printf("\n");
  }
}

#endif  // _I20_TCC_IOSET_
