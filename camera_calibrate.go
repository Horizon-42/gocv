package gocv

/*
#include <stdlib.h>
#include "camera_calibrate.h"
*/
import "C"
import "image"

// 内参外参数标定相关函数

func GetInternalMat(pics []Mat, patternSize image.Point, cameraMatrix, distCoffs *Mat) float64 {
	cMatArray:=make([]C.Mat, len(pics))

	for i, r := range pics {
		cMatArray[i] = r.p
	}

	cMats := C.struct_Mats{
		mats:   (*C.Mat)(&cMatArray[0]),
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

	cPicArray := make([]C.Mat, 2)
	cCameraMatArray := make([]C.Mat, 2)
	cDistCoffsArray := make([]C.Mat, 2)

	for i := 0; i < 2; i++ {
		cPicArray[i] = pics[i].p
		cCameraMatArray[i] = cameraMatrix[i].p
		cDistCoffsArray[i] = distCoffs[i].p
	}

	cPics := C.struct_Mats{
		mats:   (*C.Mat)(&cPicArray[0]),
		length: C.int(2),
	}
	cCameraMats := C.struct_Mats{
		mats:   (*C.Mat)(&cCameraMatArray[0]),
		length: C.int(2),
	}
	cDistCoffs := C.struct_Mats{
		mats:   (*C.Mat)(&cDistCoffsArray[0]),
		length: C.int(2)),
	}
	sz := C.struct_Size{
		width:  C.int(patternSize.X),
		height: C.int(patternSize.Y),
	}
	return bool(C.GetBMat(cPics, cCameraMats, cDistCoffs, sz, B.Ptr()))
}
