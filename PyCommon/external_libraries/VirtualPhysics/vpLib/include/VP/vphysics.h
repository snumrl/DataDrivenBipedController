/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VIRTUAL_PHYSICS
#define VIRTUAL_PHYSICS

#include <VP/vpDataType.h>
#include <VP/vpMaterial.h>
#include <VP/vpGeom.h>
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpWJoint.h>
#include <VP/vpRJoint.h>
#include <VP/vpPJoint.h>
#include <VP/vpUJoint.h>
#include <VP/vpSJoint.h>
#include <VP/vpBJoint.h>
#include <VP/vp1DOFJoint.h>
#include <VP/vpNDOFJoint.h>
#include <VP/vpSpring.h>
#include <VP/vpSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpTimer.h>

/*! \mainpage VirtualPhysics

\section Overview
<center>
<table cellSpacing="1" cellPadding="0" border="0">
	<tr>
		<td><img hspace=1 src="\vp_site/silo.png"></td>
	</tr>
	<tr>
		<td><center>Realtime(30fps) simulation of 1560 rigid bodies on P4 2.4GHz</center></td>
	</tr>
</table>
</center>
<br><br>
<em>VirtualPhysics</em> is a free C++ 
library for realtime multi-body dynamics simulation. Dynamics simulation can be 
used in wide area of mechanical engineering, computer graphics animation, 
virtual reality and more. <em>VirtualPhysics</em> is designed for simulating 
behaviors of articulated rigid body systems in realtime. Using <em>Virtual 
Physics</em>, programmers can easily implement simulation based applications 
with moderate understanding of related physics and mathematics. 
<br>
\section feature Features of VirtualPhysics
<ul>
  <li>Realtime : <em>VirtualPhysics</em> uses Lie Group recursive dynamics 
  algorithms. It is fast and stable enough for realtime applications such as 
  an adaptive controller for robotic systems, interactive games and VR contents.</li>
  <li>User defined joint : You can design your own joint to constrain rigid bodies.
  For example, you can define a curvy prismatic joint to model a roller coaster rail.</li>
  <li>Collision detection and response : <em>VirtualPhysics</em> separates 
  collision detection and response. If you have your own collision detection 
  module, <em>VirtualPhysics</em> can easily cooperate with your module to make 
  a collision response. A built-in collision detection module supports collision 
  between a few kinds of primitive geometries such as box, sphere and cylinder. 
  Collision response in <em>VirtualPhysics</em> can handle multiple impulsive 
  collision and resting contact under frictional conditions.</li>
  <li>Joint limit : <em>VirtualPhysics</em> resolves joint limit in a 
  collision-like manner. If a joint limit is defined with a coefficient of 
  restitution, the angle will not only be restricted in its limit, but also the 
  generated behavior is physically reasonable.</li>
  <li>Object oriented data structures : Programmers familiar with C++ and object 
  orientation can easily understand, modify and improve data structures of 
  <em>VirtualPhysics</em>.</li>
  <li>Runs on various platforms such as win32, Linux, Sun OS and IRIX.</li>
</ul>
<br>
\ref _getting_started "Getting Started"
<br>
\ref _user_manual "User Manual"
<br>
\ref _download "Download"
*/

/*! \page _getting_started Getting Started
Getting Started
*/

/*! \page _user_manual User Manual
User Manual
*/

/*! \page _download Donwload
<ul>
<li>Currently <em>VirtualPhysics</em> is developed on Microsoft Visual Studio 2008.
So you need VS2008 to complie your applications and link with <b>vpLib.lib</b>.
<li>All examples use <em>vpRenderer</em>, a simple rendering framework. 
<em>vpRenderer</em> depend on <a href="http://www.glew.org">glew</a> and <a href="http://www.xmission.com/~nate/glut.html">glut</a> libraries.
<li>Please feel free to tell us any bugs, problems and suggestions. You can reach me at jwkim@imrc.kist.re.kr.
<li>The latest release is <a href="VirtualPhysics_v09.zip">VirtualPhysics v0.9</a>.
*/

/** \example pendulum.cpp
Here is a tutorial for beginners. 
The followings are typical procedures to construct and simulate dynamic systems. 
<ol>
	<li>Declare a world, rigid bodies and joints.</li>
	<li>Construct a shape of a rigid body using vpBody::AddGeometry</li>
	<li>Connect bodies with joints using a vpBody::SetJoint. 
	Each joint connect two adjacent bodies. A body can be attached to several joints.</li>
	<li>If you want, you can decalare a body as a ground using a vpBody::SetGround.
	The body is assume to be a ground which means the body will not move.</li>
	<li>Add bodies using vpWorld::AddBody.</li>
	<li>Initialize your world using vpWorld::Initialize.</li>
	<li>Make your world moving by calling vpWorld::StepAhead repeatedly.</li>
</ol>
<img src="\vp_site/pendulum.jpg">
*/

#endif
