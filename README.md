# Data-Driven Biped Controller

Related Author: [Yoonsang Lee](http://mrl.snu.ac.kr/~yslee)

Related Publication : [Siggraph 2010](http://mrl.snu.ac.kr/research/ProjectDataDrivenBiped/index.html)


This software is an implementation of a dynamic controller to physically simulate under-actuated three-dimensional full-body biped locomotion. The controller takes motion capture reference data to reproduce realistic human locomotion through realtime physically based simulation. Examples includes turning, spining, and walking while steering its direction interactively. 

---------------------
##Install

Install following packages : 
python-2.5.2.msi
psyco-1.6.win32-py25.exe
pyFltk-1.1.2.win32-py2.5.exe
numpy-1.2.0-win32-superpack-python2.5.exe
PyOpenGL-3.0.0.win32.exe
matplotlib-0.91.4.win32-py2.5.exe


##Examples

Example scripts are starts with 'main_' and exist in the Walking/.
You can change running example of each file by commenting/uncommenting function-call lines at the end of the file.

main_CharacterExamples : Walking of different mass distrition / body part length characters
main_InteractiveExample2 : Interactive control example
main_VariousExamples : Uphill/downhill walking, turning, spinning
main_PushExample : Pushing by external force example
main_TrackingExamples2 : Various tracking examples


##Run

You can run examples just by double-click each example script file in the windows explorer, or you can open the cmd window, change the directory to the Walking/ and type a command like 'C:\Python25\python main_CharacterExamples.py'


##Complie

C++ code part of this project was compiled with Visual Studio .NET 2003 (VC++ 7.1) with Boost python 1.35.0 and not tested with other versions or compilers.
You can build the core lib by opening PyCommon/modules/modules.sln in the Visual Stuidio and compile the "modules" proejct with "Release" configuration, and you can build the pyd extensions by running PyCommon/modules/setup.py like: "python setup.py build" in the command line window.


----------------------

This code was written for and is being made available to accompany the ACM Transactions on Graphics (SIGGRAPH 2010), Vol. 29, No. 4, Article 129, July 2010  "Data-Driven Biped Control"

For updates to this code, please check our website, mrl.snu.ac.kr. 
Yoonsang Lee can be contacted at yslee@mrl.snu.ac.kr
