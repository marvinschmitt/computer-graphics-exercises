#include <Raytracer/Raytracer.h>

using namespace Raytracer;
using namespace Raytracer::Scenes;
using namespace Raytracer::Objects;

Sphere::Sphere(const float3 &center, float radius)
{
	this->center = center;
	this->radius = radius;
	this->radius2 = radius * radius;
	this->material = NULL;
}

Sphere::Sphere(const float3 &center, float radius, Material *material)
{
	this->center = center;
	this->radius = radius;
	this->radius2 = radius * radius;
	this->material = material;
}

void Sphere::GetExtent(float3 &min, float3 &max) const
{
	min = center - radius;
	max = center + radius;
};

void Sphere::GetIntersection(const Ray &ray, float distance, Intersection &intersection) const
{
	// TODO: Determine position, viewing direction, normals, and material at the intersection point 
	// of the given ray and the given distance.
	// I.e., insert those values into the intersection struct (3rd parameter)
	// The parameter "distance" (2nd parameter) contains the distance to the ray origin - intersection point.
}

bool Sphere::HitTest(const Ray &ray, RayHit &hit) const
{
	hit.Set(&ray, 0, NULL);
	// TODO: Implement the intersection. If the ray hits the sphere set "hit = this;", 
	// set the distance to the next intersection point, and return true (see class RayHit).
	return false;
}
