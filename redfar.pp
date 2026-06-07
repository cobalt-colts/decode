{
  "startPoint": {
    "x": 82.5,
    "y": 9.5,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "launch1-line",
      "name": "launch1",
      "endPoint": {
        "x": 82.5,
        "y": 15.0,
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
      "id": "cycle-1",
      "name": "cycle_curve",
      "endPoint": {
        "x": 131.572,
        "y": 10.145,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 93.768,
          "y": 29.936
        },
        {
          "x": 99.063,
          "y": 9.335
        }
      ],
      "color": "#8D7B8A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "cycle-2",
      "name": "cycle_bump1",
      "endPoint": {
        "x": 120.0,
        "y": 11.0,
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
      "id": "cycle-3",
      "name": "cycle_bump2",
      "endPoint": {
        "x": 131.572,
        "y": 10.145,
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
      "id": "cycle-4",
      "name": "cycle_return",
      "endPoint": {
        "x": 82.5,
        "y": 15.0,
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
      "id": "farline-1",
      "name": "farline_out",
      "endPoint": {
        "x": 127.0,
        "y": 34.75,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 92.0,
          "y": 37.0
        }
      ],
      "color": "#DC6776",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "farline-2",
      "name": "farline_return",
      "endPoint": {
        "x": 82.5,
        "y": 15.0,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [
        {
          "x": 92.0,
          "y": 37.0
        }
      ],
      "color": "#DC6776",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "leave-1",
      "name": "leave",
      "endPoint": {
        "x": 103.5,
        "y": 32.5,
        "heading": "linear",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#22C55E",
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
      "lineId": "launch1-line"
    },
    {
      "kind": "path",
      "lineId": "cycle-1"
    },
    {
      "kind": "path",
      "lineId": "cycle-2"
    },
    {
      "kind": "path",
      "lineId": "cycle-3"
    },
    {
      "kind": "path",
      "lineId": "cycle-4"
    },
    {
      "kind": "path",
      "lineId": "farline-1"
    },
    {
      "kind": "path",
      "lineId": "farline-2"
    },
    {
      "kind": "path",
      "lineId": "leave-1"
    }
  ],
  "pathChains": [
    {
      "id": "chain-launch1",
      "name": "launch1",
      "color": "#D55DDD",
      "lineIds": [
        "launch1-line"
      ]
    },
    {
      "id": "chain-cycle",
      "name": "cycle",
      "color": "#8D7B8A",
      "lineIds": [
        "cycle-1",
        "cycle-2",
        "cycle-3",
        "cycle-4"
      ]
    },
    {
      "id": "chain-farline",
      "name": "farline",
      "color": "#DC6776",
      "lineIds": [
        "farline-1",
        "farline-2"
      ]
    },
    {
      "id": "chain-leave",
      "name": "leave",
      "color": "#22C55E",
      "lineIds": [
        "leave-1"
      ]
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-06-06T00:00:00.000Z"
}