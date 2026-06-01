{
  "startPoint": {
    "x": 82.5,
    "y": 9.5,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "mpq1eaca-igyq75",
      "name": "launch1",
      "endPoint": {
        "x": 82.5,
        "y": 15,
        "heading": "constant",
        "reverse": false,
        "degrees": 90
      },
      "controlPoints": [],
      "color": "#D55DDD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpvosbzx-stjaez",
      "name": "cornerline",
      "endPoint": {
        "x": 131.57229524772495,
        "y": 10.144590495449954,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 93.76790507219724,
          "y": 29.936292943569587
        },
        {
          "x": 99.06315279717197,
          "y": 9.335180708989203
        }
      ],
      "color": "#8D7B8A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpvr0qv7-d5dq20",
      "name": "bump1",
      "endPoint": {
        "x": 120,
        "y": 11,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [],
      "color": "#8D7B8A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpvr51y1-1hn3fe",
      "name": "bump2",
      "endPoint": {
        "x": 131.57229524772495,
        "y": 10.144590495449954,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#8D7B8A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpvov3hn-0r8j24",
      "name": "launch2",
      "endPoint": {
        "x": 82.5,
        "y": 15,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [],
      "color": "#8D7B8A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpvp29ga-asayjp",
      "name": "line",
      "endPoint": {
        "x": 127,
        "y": 34.75,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 92,
          "y": 37
        }
      ],
      "color": "#DC6776",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpvp3jri-70n688",
      "name": "launch3",
      "endPoint": {
        "x": 82.5,
        "y": 15,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [
        {
          "x": 92,
          "y": 37
        }
      ],
      "color": "#DC6776",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 120,
          "y": 141.5
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "mpq1eaca-igyq75"
    },
    {
      "kind": "path",
      "lineId": "mpvosbzx-stjaez"
    },
    {
      "kind": "path",
      "lineId": "mpvr0qv7-d5dq20"
    },
    {
      "kind": "path",
      "lineId": "mpvr51y1-1hn3fe"
    },
    {
      "kind": "path",
      "lineId": "mpvov3hn-0r8j24"
    },
    {
      "kind": "path",
      "lineId": "mpvp29ga-asayjp"
    },
    {
      "kind": "path",
      "lineId": "mpvp3jri-70n688"
    }
  ],
  "pathChains": [
    {
      "id": "mppzv00u-zrozsn",
      "name": "launch1",
      "color": "#D55DDD",
      "lineIds": [
        "mpq1eaca-igyq75"
      ]
    },
    {
      "id": "mpvosrp8-ynwglb",
      "name": "cycle",
      "color": "#8D7B8A",
      "lineIds": [
        "mpvosbzx-stjaez",
        "mpvr0qv7-d5dq20",
        "mpvr51y1-1hn3fe",
        "mpvov3hn-0r8j24"
      ]
    },
    {
      "id": "mpvp2cx7-acdcw0",
      "name": "farline",
      "color": "#DC6776",
      "lineIds": [
        "mpvp29ga-asayjp",
        "mpvp3jri-70n688"
      ]
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-06-01T22:07:52.258Z"
}
