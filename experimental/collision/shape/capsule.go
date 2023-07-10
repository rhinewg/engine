// Copyright 2016 The G3N Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package shape

import (
	"github.com/g3n/engine/math32"
)

type Capsule struct {
	Radius float32
	Height float32
}

func NewCapsule(radius, height float32) *Capsule {
	return &Capsule{radius, height}
}

func (c *Capsule) GetRadius() float32 {
	return c.Radius
}

func (c *Capsule) GetHeight() float32 {
	return c.Height
}

func (c *Capsule) SetRadius(radius float32) {
	c.Radius = radius
}

func (c *Capsule) SetHeight(height float32) {
	c.Height = height
}

func (c *Capsule) Volume() float32 {
	cylinderVolume := math32.Pi * math32.Pow(c.Radius, 2) * c.Height
	sphereVolume := (4.0 / 3.0) * math32.Pi * math32.Pow(c.Radius, 3)
	return cylinderVolume + sphereVolume
}

func (c *Capsule) Area() float32 {
	//计算胶囊体表面积
	cylinderSurfaceArea := 2 * math32.Pi * c.Radius * c.Height
	sphereSurfaceArea := 2 * math32.Pi * math32.Pow(c.Radius, 2)
	return cylinderSurfaceArea + sphereSurfaceArea
}

func (c *Capsule) BoundingBox() math32.Box3 {
	//计算胶囊体外接立方体
	halfHeight := c.Height / 2
	minX := -c.Radius
	maxX := c.Radius
	minY := -halfHeight - c.Radius
	maxY := halfHeight + c.Radius
	minZ := -c.Radius
	maxZ := c.Radius
	return math32.Box3{math32.Vector3{minX, minY, minZ}, math32.Vector3{maxX, maxY, maxZ}}
}

func (c *Capsule) BoundSphere() math32.Sphere {
	//计算胶囊体外接球
	raidus := c.Height/2 + c.Radius
	return *math32.NewSphere(math32.NewVec3(), raidus)
}

func (c *Capsule) RotationalInertiaX() float32 {
	cylinderInertia := (1.0 / 12.0) * math32.Pi * c.Height * (3*math32.Pow(c.Radius, 2) + math32.Pow(c.Height, 2))
	sphereInertia := (2.0 / 5.0) * math32.Pi * math32.Pow(c.Radius, 2) * math32.Pow(c.Radius/2+c.Height, 2)
	return cylinderInertia + sphereInertia
}
func (c *Capsule) RotationalInertiaY() float32 {
	cylinderInertia := (1.0 / 12.0) * math32.Pi * c.Height * (3*math32.Pow(c.Radius, 2) + math32.Pow(c.Height, 2))
	sphereInertia := (2.0 / 5.0) * math32.Pi * math32.Pow(c.Radius, 2) * math32.Pow(c.Radius/2+c.Height, 2)
	return cylinderInertia + sphereInertia
}
func (c *Capsule) RotationalInertiaZ() float32 {
	return (1.0 / 2.0) * math32.Pi * math32.Pow(c.Radius, 2) * c.Height
}

// RotationalInertia computes and returns the rotational inertia of the analytical collision sphere.
func (c *Capsule) RotationalInertia(mass float32) math32.Matrix3 {
	Ix := c.RotationalInertiaX() * mass
	Iy := c.RotationalInertiaY() * mass
	Iz := c.RotationalInertiaZ() * mass
	return math32.Matrix3{
		Ix, 0, 0,
		0, Iy, 0,
		0, 0, Iz,
	}
}

func (c *Capsule) ProjectOntoAxis(localAxis *math32.Vector3) (float32, float32) {

	//计算胶囊体在轴上的投影
	halfHeight := c.Height / 2
	projection := math32.Abs(localAxis.Dot(&math32.Vector3{0, halfHeight, 0}))
	projection += math32.Abs(localAxis.Dot(&math32.Vector3{0, -halfHeight, 0}))
	projection += math32.Abs(localAxis.Dot(&math32.Vector3{halfHeight, 0, 0}))
	projection += math32.Abs(localAxis.Dot(&math32.Vector3{-halfHeight, 0, 0}))
	projection += math32.Abs(localAxis.Dot(&math32.Vector3{0, 0, halfHeight}))
	projection += math32.Abs(localAxis.Dot(&math32.Vector3{0, 0, -halfHeight}))
	projection += c.Radius

	//计算胶囊体在轴上的投影的最小值和最大值
	min := -projection
	max := projection

	return min, max
}
