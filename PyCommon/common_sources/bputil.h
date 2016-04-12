// +-------------------------------------------------------------------------
// | bputil.h
// |
// | Author: Yoonsang Lee
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Yoonsang Lee 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the DataDrivenBipedController.
// |    DataDrivenBipedController is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------

#pragma once

//#include <boost/python.hpp>
//#include <python.h>
//using namespace boost::python;

#define XD(v)	((extract<double>)(v))
#define XC(v)	((extract<char*>)(v))
#define XS(v)	((extract<string>)(v))
#define XF(v)	((extract<float>)(v))
#define XI(v)	((extract<int>)(v))
#define XB(v)	((extract<bool>)(v))



//inline void printSO3(const object& SO3)
//{
//	cout << "[ ";\
//	for(int i=0; i<len(SO3); ++i)\
//	{\
//		if(i>0)\
//			cout << "  ";\
//		for(int j=0; j<len(SO3[i]); ++j)\
//			cout << XD(SO3[i][j]) << ' ';\
//		if(i==len(SO3)-1)\
//			cout << "]";\
//		cout << endl;\
//	}\
//}

//inline void Tokenize(const string& str,
//                      vector<string>& tokens,
//                      const string& delimiters = " ")
//{
//    // 맨 첫 글자가 구분자인 경우 무시
//    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
//    // 구분자가 아닌 첫 글자를 찾는다
//    string::size_type pos     = str.find_first_of(delimiters, lastPos);
//
//    while (string::npos != pos || string::npos != lastPos)
//    {
//        // token을 찾았으니 vector에 추가한다
//        tokens.push_back(str.substr(lastPos, pos - lastPos));
//        // 구분자를 뛰어넘는다.  "not_of"에 주의하라
//        lastPos = str.find_first_not_of(delimiters, pos);
//        // 다음 구분자가 아닌 글자를 찾는다
//        pos = str.find_first_of(delimiters, lastPos);
//    }
//}
//