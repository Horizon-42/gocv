package gocv

/*
#include <stdlib.h>
#include "camera_calibrate.h"
*/
import "C"
import "image"

// 内参外参数标定相关函数

func GetInternalMat(pics []Mat, patternSize image.Point, cameraMatrix, distCoffs *Mat) float64 {
	cMats := C.struct_Mats{
		mats:   (*C.Mat)(&pics[0]),
		length: C.int(len(pics)),
	}

	sz := C.struct_Size{
		width:  C.int(patternSize.X),
		height: C.int(patternSize.Y),
	}
	return float64(C.GetInternalMat(cMats, sz, cameraMatrix.Ptr(), distCoffs.Ptr()))
}

func GetBMat(pics, cameraMatrix, distCoffs []Mat, patternSize image.Point, B *Mat) bool {
	if len(pics) != 2 || len(cameraMatrix) != 2 || len(distCoffs) != 2 {
		panic("input error")
	}
	cPics := C.struct_Mats{
		mats:   (*C.Mat)(&pics[0]),
		length: C.int(len(pics)),
	}
	cCameraMats := C.struct_Mats{
		mats:   (*C.Mat)(&cameraMatrix[0]),
		length: C.int(len(cameraMatrix)),
	}
	cDistCoffs := C.struct_Mats{
		mats:   (*C.Mat)(&distCoffs[0]),
		length: C.int(len(distCoffs)),
	}
	sz := C.struct_Size{
		width:  C.int(patternSize.X),
		height: C.int(patternSize.Y),
	}
	return bool(C.GetBMat(cPics, cCameraMats, cDistCoffs, sz, B.Ptr()))
}
