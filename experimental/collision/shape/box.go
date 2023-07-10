// Copyright 2016 The G3N Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package shape

import "github.com/g3n/engine/math32"

// Box is an analytical collision Box.
type Box struct {
	length, width, height float32
}

// NewBox creates and returns a pointer to a new analytical collision Box.
func NewBox(center math32.Vector3, length, width, height float32) *Box {
	s := new(Box)
	s.length = length
	s.width = width
	s.height = height
	return s
}

// IShape =============================================================

// BoundingBox computes and returns the bounding box of the analytical collision Box.
func (s *Box) BoundingBox() math32.Box3 {
	return math32.Box3{math32.Vector3{-s.length / 2, -s.width / 2, -s.height / 2}, math32.Vector3{s.length / 2, s.width / 2, s.height / 2}}
}

// BoundingBox computes and returns the bounding Box of the analytical collision Sphere.
func (s *Box) BoundingSphere() math32.Sphere {
	radius := math32.Max(s.length, s.width)
	radius = math32.Max(radius, s.height)
	return *math32.NewSphere(math32.NewVec3(), radius)
}

// Area computes and returns the surface area of the analytical collision Box.
func (s *Box) Area() float32 {
	return 2 * (s.length*s.width + s.length*s.height + s.width*s.height)
}

// Volume computes and returns the volume of the analytical collision Box.
func (s *Box) Volume() float32 {
	return s.length * s.width * s.height
}

// RotationalInertia computes and returns the rotational inertia of the analytical collision Box.
func (s *Box) RotationalInertia(mass float32) math32.Matrix3 {
	massOverTwelve := mass / 12
	return math32.Matrix3{
		massOverTwelve * (s.width*s.width + s.height*s.height), 0, 0,
		0, massOverTwelve * (s.length*s.length + s.height*s.height), 0,
		0, 0, massOverTwelve * (s.length*s.length + s.width*s.width)}
}

// ProjectOntoAxis computes and returns the minimum and maximum distances of the analytical collision Box projected onto the specified local axis.
func (s *Box) ProjectOntoAxis(localAxis *math32.Vector3) (float32, float32) {

	// Compute the radius of the box along the given axis.
	radius := s.length*math32.Abs(localAxis.Dot(&math32.Vector3{1, 0, 0})) +
		s.width*math32.Abs(localAxis.Dot(&math32.Vector3{0, 1, 0})) +
		s.height*math32.Abs(localAxis.Dot(&math32.Vector3{0, 0, 1}))

	return -radius, radius
}
