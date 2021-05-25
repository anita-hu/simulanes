# TuSimple config, --res 1280x720
row_anchors = [160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360,
               370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570,
               580, 590, 600, 610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710]
               
# CULane config, --res 1640x590
# row_anchors = [260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460,
#                470, 480, 490, 500, 510, 520, 530]

lane_classes = {
    0: "none",  # default value
    1: "dashed white",
    2: "dashed yellow",
    3: "solid white",
    4: "solid yellow",
    5: "double dashed white",
    6: "double dashed yellow",
    7: "double solid white",
    8: "double solid yellow",
    9: "solid dashed white",
    10: "solid dashed yellow",
    11: "dashed solid white",
    12: "dashed solid yellow",
    13: "botts dots",
    14: "curb",
}
