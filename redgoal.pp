{
  "startPoint": {
    "x": 120,
    "y": 123,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-c4bqcruv9sw",
      "name": "launch1",
      "endPoint": {
        "x": 91,
        "y": 92,
        "heading": "linear",
        "startDeg": 37,
        "endDeg": 50,
        "degrees": 37
      },
      "controlPoints": [],
      "color": "#666CAB",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpddrjfy-38e20v",
      "name": "line2",
      "endPoint": {
        "x": 123,
        "y": 65,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 100,
          "y": 65
        }
      ],
      "color": "#788755",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpddvjyn-ha3hw3",
      "name": "launch2",
      "endPoint": {
        "x": 91,
        "y": 92,
        "heading": "tangential",
        "reverse": true,
        "startDeg": 0,
        "endDeg": 37
      },
      "controlPoints": [
        {
          "x": 100,
          "y": 65
        }
      ],
      "color": "#788755",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpojw6xg-heuivw",
      "name": "gate1",
      "endPoint": {
        "x": 130,
        "y": 65,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 107.92618806875632,
          "y": 57.612740141557126
        }
      ],
      "color": "#59CCAB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpojxoh0-qtj1pb",
      "name": "gate2",
      "endPoint": {
        "x": 134.88831548809904,
        "y": 57.54123032612182,
        "heading": "linear",
        "reverse": false,
        "degrees": 45,
        "startDeg": 19,
        "endDeg": 45
      },
      "controlPoints": [
        {
          "x": 129.4411243770121,
          "y": 58.69983659885464
        }
      ],
      "color": "#666CAB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mpok4yq2-yllqvk",
      "name": "returnToLaunch",
      "endPoint": {
        "x": 91,
        "y": 92,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [
        {
          "x": 102.21513853272496,
          "y": 59.03097916710539
        }
      ],
      "color": "#666CAB",
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
      "lineId": "line-c4bqcruv9sw"
    },
    {
      "kind": "path",
      "lineId": "mpddrjfy-38e20v"
    },
    {
      "kind": "path",
      "lineId": "mpddvjyn-ha3hw3"
    },
    {
      "kind": "path",
      "lineId": "mpojw6xg-heuivw"
    },
    {
      "kind": "path",
      "lineId": "mpojxoh0-qtj1pb"
    },
    {
      "kind": "wait",
      "id": "mpok4rhe-xmwoja",
      "name": "Wait",
      "durationMs": 1000,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mpok4yq2-yllqvk"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mpddcgr3-molscm",
      "name": "launch1",
      "color": "#666CAB",
      "lineIds": [
        "line-c4bqcruv9sw"
      ]
    },
    {
      "id": "mpddklni-o6vchn",
      "name": "line1",
      "color": "#788755",
      "lineIds": [
        "mpddrjfy-38e20v",
        "mpddvjyn-ha3hw3"
      ]
    },
    {
      "id": "mpojvvgv-4vewov",
      "name": "gate",
      "color": "#59CCAB",
      "lineIds": [
        "mpojw6xg-heuivw",
        "mpojxoh0-qtj1pb"
      ]
    },
    {
      "id": "chain-mpok4yq2-yllqvk",
      "name": "returnToLaunch",
      "color": "#666CAB",
      "lineIds": [
        "mpok4yq2-yllqvk"
      ]
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-05-27T21:21:35.159Z"
}
