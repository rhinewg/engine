// Copyright 2016 The G3N Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package shape

import (
	"github.com/g3n/engine/math32"
	"math"
)

type Cylinder struct {
	Radius float32
	Height float32
}

func NewCylinder(radius, height float32) *Cylinder {
	return &Cylinder{radius, height}
}

func (c *Cylinder) GetRadius() float32 {
	return c.Radius
}

func (c *Cylinder) GetHeight() float32 {
	return c.Height
}

func (c *Cylinder) SetRadius(radius float32) {
	c.Radius = radius
}

func (c *Cylinder) SetHeight(height float32) {
	c.Height = height
}

func (c *Cylinder) Volume() float32 {
	return math.Pi * c.Radius * c.Radius * c.Height
}

func (c *Cylinder) Area() float32 {
	return 2 * math.Pi * c.Radius * (c.Radius + c.Height)
}

func (s *Cylinder) BoundingBox() math32.Box3 {
	halfHeight := s.Height / 2
	minX := -s.Radius
	maxX := s.Radius
	minY := -halfHeight
	maxY := halfHeight
	minZ := -s.Radius
	maxZ := s.Radius
	return math32.Box3{math32.Vector3{minX, minY, minZ}, math32.Vector3{maxX, maxY, maxZ}}
}

func (c *Cylinder) BoundSphere() math32.Sphere {
	raidus := math32.Max(c.Radius, c.Height/2)
	return *math32.NewSphere(math32.NewVec3(), raidus)
}

func (c *Cylinder) RotationalInertia(mass float32) math32.Matrix3 {
	//计算X轴惯性矩
	Ix := (1.0 / 12.0) * mass * (3*c.Radius*c.Radius + c.Height*c.Height)
	//计算Y轴惯性矩
	Iy := (1.0 / 2.0) * mass * c.Radius * c.Radius
	//计算Z轴惯性矩
	Iz := Ix

	return math32.Matrix3{
		Ix, 0, 0,
		0, Iy, 0,
		0, 0, Iz,
	}
}

func (c *Cylinder) ProjectOntoAxis(localAxis *math32.Vector3) (float32, float32) {

	halfHeight := c.Height / 2
	radius := c.Radius
	absDot := math32.Abs(localAxis.Y)

	// Project the half height onto the axis
	halfHeightProj := halfHeight * absDot

	// Project the radius onto the axis
	radiusProj := radius * math32.Sqrt(1-absDot*absDot)

	return -halfHeightProj, halfHeightProj + radiusProj
}
