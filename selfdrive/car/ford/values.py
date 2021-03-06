from selfdrive.car import dbc_dict

MAX_ANGLE = 87.  # make sure we never command the extremes (0xfff) which cause latching fault

class CAR:
  FUSION = "FORD FUSION 2018"
  EXPLORER = "FORD EXPLORER 2013"

FINGERPRINTS = {
  CAR.FUSION: [{
    71: 8, 74: 8, 75: 8, 76: 8, 90: 8, 92: 8, 93: 8, 118: 8, 119: 8, 120: 8, 125: 8, 129: 8, 130: 8, 131: 8, 132: 8, 133: 8, 145: 8, 146: 8, 357: 8, 359: 8, 360: 8, 361: 8, 376: 8, 390: 8, 391: 8, 392: 8, 394: 8, 512: 8, 514: 8, 516: 8, 531: 8, 532: 8, 534: 8, 535: 8, 560: 8, 578: 8, 604: 8, 613: 8, 673: 8, 827: 8, 848: 8, 934: 8, 935: 8, 936: 8, 947: 8, 963: 8, 970: 8, 972: 8, 973: 8, 984: 8, 992: 8, 994: 8, 997: 8, 998: 8, 1003: 8, 1034: 8, 1045: 8, 1046: 8, 1053: 8, 1054: 8, 1058: 8, 1059: 8, 1068: 8, 1072: 8, 1073: 8, 1082: 8, 1107: 8, 1108: 8, 1109: 8, 1110: 8, 1200: 8, 1427: 8, 1430: 8, 1438: 8, 1459: 8
  },
  #F150
  {
    74: 8, 75: 8, 76: 8, 92: 8, 118: 8, 119: 8, 120: 8, 125: 8, 129: 8, 130: 8, 131: 8, 133: 8, 145: 8, 146: 8, 357: 8, 359: 8, 370: 8, 376: 8, 390: 8, 391: 8, 392: 8, 394: 8, 512: 8, 514: 8, 516: 8, 531: 8, 532: 8, 534: 8, 535: 8, 560: 8, 578: 8, 611: 8, 613: 8, 673: 8, 827: 8, 845: 8, 848: 8, 850: 8, 934: 8, 935: 8, 936: 8, 938: 8, 939: 8, 945: 8, 947: 8, 961: 8, 962: 8, 963: 8, 970: 8, 972: 8, 973: 8, 979: 8, 980: 8, 983: 8, 984: 8, 985: 8, 992: 8, 994: 8, 997: 8, 998: 8, 1003: 8, 1006: 8, 1034: 8, 1042: 8, 1045: 8, 1046: 8, 1047: 8, 1053: 8, 1054: 8, 1056: 8, 1058: 8, 1059: 8, 1068: 8, 1072: 8, 1073: 8, 1080: 8, 1082: 8, 1090: 8, 1091: 8, 1093: 8, 1102: 8, 1105: 8, 1107: 8, 1108: 8, 1109: 8, 1110: 8, 1114: 8, 1122: 8, 1126: 8, 1186: 8, 1200: 8, 1430: 8, 1438: 8, 1441: 8, 1459: 8, 1461: 8, 1472: 8, 1609: 8, 1611: 8, 1798: 8, 1824: 8, 1888: 8, 1896: 8
  }],
  CAR.EXPLORER: [{
    65: 8, 66: 8, 71: 8, 74: 8, 75: 8, 116: 8, 117: 8, 124: 8, 129: 8, 130: 8, 131: 8, 132: 8, 145: 8, 146: 8, 258: 8, 259: 8, 264: 8, 266: 8, 267: 8, 292: 8, 293: 8, 337: 8, 340: 8, 342: 8, 345: 8, 357: 8, 389: 8, 392: 8, 393: 8, 394: 8, 512: 8, 513: 8, 529: 8, 533: 8, 534: 8, 557: 8, 560: 8, 576: 8, 592: 8, 597: 8, 608: 8, 613: 8, 806: 8, 832: 8, 842: 8, 843: 8, 844: 8, 845: 8, 846: 8, 848: 8, 849: 8, 850: 8, 853: 8, 854: 8, 855: 8, 856: 8, 857: 8, 860: 8, 906: 8, 909: 8, 936: 8, 937: 8, 942: 8, 947: 8, 948: 8, 949: 8, 955: 8, 957: 8, 961: 8, 963: 8, 967: 8, 970: 8, 971: 8, 972: 8, 984: 8, 986: 8, 992: 8, 993: 8, 994: 8, 995: 8, 997: 8, 998: 8, 999: 8, 1001: 8, 1002: 8, 1003: 8, 1004: 8, 1034: 8, 1036: 8, 1044: 8, 1045: 8, 1046: 8, 1047: 8, 1056: 8, 1058: 8, 1067: 8, 1068: 8, 1069: 8, 1072: 8, 1075: 8, 1107: 8, 1108: 8, 1109: 8, 1125: 8, 1126: 8, 1127: 8, 1152: 8, 1409: 8, 1425: 8, 1459: 8, 1486: 8, 1506: 8
  }],
}

DBC = {
  CAR.FUSION: dbc_dict('ford_fusion_2018_pt', 'ford_fusion_2018_adas'),
  CAR.EXPLORER: dbc_dict('ford_explorer_2013', 'ford_explorer_2013'),
}
