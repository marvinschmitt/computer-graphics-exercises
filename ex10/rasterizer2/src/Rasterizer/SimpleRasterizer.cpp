#include <algorithm>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/matrix_inverse.hpp>

#define RAYTRACER_USE_FOREACH
#include <Raytracer/Raytracer.h>

#include <Rasterizer/SimpleRasterizer.h>

using namespace std;
using namespace glm;
using namespace Rasterizer;
using namespace Raytracer;
using namespace Raytracer::Objects;
using namespace Raytracer::Scenes;

SimpleRasterizer::SimpleRasterizer()
{
  ambientLight = vec3(0.01f);
}

bool SimpleRasterizer::CompareVertexScanline(const vec3& p1, const vec3& p2)
{
    return (p1.x < p2.x);
}

bool SimpleRasterizer::CompareTriangle(const Triangle &t1, const Triangle &t2)
{
  // These aren't actually the mean values, but since both are off by a constant factor (3),
  // this inequation is equivalent.
  return (t1.position[0].z + t1.position[1].z + t1.position[2].z >
    t2.position[0].z + t2.position[1].z + t2.position[2].z);
}


void SimpleRasterizer::DrawSpan(int x1, int x2, int y, float z1, float z2, const vec3 &color1, const vec3 &color2)
{
    if (x1 > x2)
    {
        int temp = x1;
        x1 = x2;
        x2 = temp;
    }

    float lambda;
    vec3 color;

    for (int x = x1; x < x2; ++x)  // x<x2 to not draw right pixel
    {   
        color = color1 + (float)(x - x1) / (x2 - x1) * (color2 - color1);
        if ((x > 0) && (x < image->GetWidth()) && (y > 0) && (y < image->GetHeight()))
        {
            image->SetPixel(x, y, color);
        }
    }
  // TODO Aufgabe 2: Ersetzen des Zeichnens der Eckpunkte 
  // durch Dreiecksrasterisierer, Gouraud Shading, [z-Buffering]
}

float SimpleRasterizer::delta_x(const Triangle& t, int next, int cur) {
    float x_t_prime = t.position[next % 3].x;
    float x_t = t.position[cur % 3].x;
    float y_t_prime = t.position[next % 3].y;
    float y_t = t.position[cur % 3].y;
    float delta = (x_t_prime - x_t) / (y_t_prime - y_t);
    return delta;
}

void SimpleRasterizer::DrawTriangle(const Triangle &t)
{

  //for (int i = 0; i < 3; ++i)
  //{
  //  int x = (int)t.position[i].x;
  //  int y = (int)t.position[i].y;
  //  if ((x > 0) && (x < image->GetWidth()) && (y > 0) && (y < image->GetHeight()))
  //  {
  //    image->SetPixel(x, y, t.color[i]);
  //  }
  //}
  // 
  
    // Create triangle tr that has sorted vertices of t. We can't sort in t since it's const in the provided code
    vector<int> idx{ 0, 1, 2 };
    sort(idx.begin(), idx.end(), [&](int i, int j) {return t.position[i].y > t.position[j].y; });

    Triangle tr;
    for (int i = 0; i < 3; ++i) {
       int t_idx = idx[i];
       tr.SetVertex(i, t.position[t_idx], t.normal[t_idx], t.color[t_idx]);
    }

    // somehow, INFINITY didn't work...
    int y_t = 100000;
    int y_b = -100000;
    int idx_t = -1;
    int idx_b = -1;

    for (int i = 0; i < 3; ++i) {
        int y = (int)tr.position[i].y;
        if (y > y_b) { 
            y_b = y; 
            idx_b = i;
        }
        if (y < y_t) { 
            y_t = y; 
            idx_t = i;
        }
    }


    vec3 P_t = tr.position[idx_t];
    float z = (tr.position[0].z + tr.position[0].z + tr.position[0].z) / 3;
    int y = y_t;
    float x_l = P_t.x;
    float x_r = P_t.x;
    int cur_l = idx_t;
    int next_l = idx_t - 1;
    int cur_r = idx_t;
    int next_r = idx_t + 1;


    vec3 color_l{ 1.0, 0.0, 0.5 };
    vec3 color_r{ 0.0, 0.5, 1.0 };

    // main scanline algorithm
    do {
        DrawSpan((int)x_l, (int)x_r, y, z, z, color_l, color_r);
        y++;
        x_l += delta_x(tr, next_l, cur_l);
        x_r += delta_x(tr, next_r, cur_r);
        
        if (y >= (int)tr.position[next_l % 3].y) {
            cur_l = next_l;
            next_l--;
        }
        if (y >= (int)tr.position[next_r % 3].y) {
            cur_r = next_r;
            next_r++;
        }
    } while (y < y_b);

  // TODO Aufgabe 2: Ersetzen des Zeichnens der Eckpunkte 
  // durch Dreiecksrasterisierer, Gouraud Shading, [z-Buffering]
}

vec3 SimpleRasterizer::LightVertex(vec4 position, vec3 normal, vec3 color)
{
  vec3 result = color * ambientLight;

  foreach (const Light *, light, lights)
  {
    vec3 intensity = vec3(1.0f);
    if ((*light)->IsInstanceOf(SceneObjectType_PointLight))
      intensity = ((PointLight *)*light)->GetIntensity();

    vec3 distance = (*light)->GetPosition() - vec3(position);
    float attenuation = 1.0f / (0.001f + dot(distance, distance));
    vec3 direction = normalize(distance);

    float lambert = glm::max(0.0f, dot(normal, direction));

    if (lambert > 0)
      result += color * lambert * attenuation * intensity;
  }

  return result;
}


void SimpleRasterizer::SortTriangles(vector<Triangle> &triangles)
{
  sort(triangles.begin(), triangles.end(), CompareTriangle);
}


void SimpleRasterizer::TransformAndLightTriangle(Triangle &t, 
                                                 const mat4 &modelTransform,
                                                 const mat4 &modelTransformNormals)
{
  for (int i = 0; i < 3; i++)
  {
    vec4 worldPos = modelTransform * vec4(t.position[i], 1.0f);
    vec3 normal = normalize(vec3(modelTransformNormals * vec4(t.normal[i], 0.0f)));

    t.color[i] = LightVertex(worldPos, normal, t.color[i]);

    vec4 pos = viewProjectionTransform * worldPos;

    // Transform to screen coordinates
    t.position[i].x = (pos.x / pos.w * 0.5f + 0.5f) * image->GetWidth();
    t.position[i].y = (pos.y / pos.w * -0.5f + 0.5f) * image->GetHeight();
    t.position[i].z = pos.z / pos.w;
  }
}


void SimpleRasterizer::RenderMesh(const Mesh *mesh)
{
  if (mesh == NULL)
    return;

  // Compute the transformation from model to world space and its transposed inverse (for the
  // normals).
  glm::mat4 modelTransform = mesh->GetGlobalTransformation();
  glm::mat4 transposedInverseMT = inverseTranspose(modelTransform);

  // Transform and light all triangles in the mesh.
  const vector<Triangle> &meshTriangles = mesh->GetTriangles();
  vector<Triangle> renderTriangles;

  foreach_c(Triangle, triangle, meshTriangles)
  {
    Triangle t = *triangle;
    TransformAndLightTriangle(t, modelTransform, transposedInverseMT);
    renderTriangles.push_back(t);
  }

  //TODO Aufgabe 2: Painter's Algorithm
  // Draw the triangles.
  foreach(Triangle, triangle, renderTriangles)
    DrawTriangle(*triangle);
}

void SimpleRasterizer::ScanObject(const Raytracer::Scenes::SceneObject *object)
{
  if (object == NULL)
    return;

  if (object->IsInstanceOf(SceneObjectType_Light))
    lights.push_back((const Light *)object);
  if (object->IsInstanceOf(SceneObjectType_Mesh))
    meshes.push_back((const Mesh *)object);

  foreach_c(SceneObject *, child, object->GetChildren())
    ScanObject(*child);
}

bool SimpleRasterizer::Render(Image &image, const Scene &scene)
{
  image.Clear(vec3(0));

  Camera *camera = scene.GetActiveCamera();
  if (camera == NULL)
    return false;

  zBuffer = new float[image.GetWidth() * image.GetHeight()];
  for (int i = 0; i < image.GetWidth() * image.GetHeight(); i++)
    zBuffer[i] = 1.0f;

  // Build lists of all lights and meshes in the scene.
  lights.clear();
  meshes.clear();
  ScanObject(&scene);

  mat4 projectionTransform = perspective(camera->GetFov(),camera->GetAspect(), 
    camera->GetNearClip(), camera->GetFarClip());
  mat4 viewTransform = lookAt(camera->GetEye(), camera->GetLookAt(), camera->GetUp());
  viewProjectionTransform = projectionTransform * viewTransform;

  // Render all meshes we found.
  this->image = &image;
  foreach(const Mesh *, mesh, meshes)
    RenderMesh(*mesh);

  delete[] zBuffer;

  return true;
}
