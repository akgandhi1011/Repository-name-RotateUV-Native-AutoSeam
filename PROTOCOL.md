# RUVSEAM protocol

`RotateUV_AutoSeam.exe input.obj output.seams distortionBound initialCut`

Output:

```text
RUVSEAM 1
COMPONENTS n
FAILED_COMPONENTS n
UNMATCHED_TRIANGLES n
DISTORTION_BOUND value
SEAMS n
SEAM vertexA vertexB
...
END
```

Open geometry boundary edges are omitted because they are already open cuts.
