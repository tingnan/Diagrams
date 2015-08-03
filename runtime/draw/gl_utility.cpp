// Copyright 2015 Native Client Authors.

#include <vector>
#include <string>
#include <iostream>

#include "utility/world_parser.h"
#include "draw/gl_utility.h"
#include "physics/node.h"

namespace diagrammar {

void ShaderErrorHandler(GLuint shader_id) {
  GLint log_size;
  glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> compileLog(log_size);
  glGetShaderInfoLog(shader_id, log_size, nullptr, compileLog.data());
  std::cerr << "Compile Error: " << shader_id << " " << compileLog.data()
            << std::endl;
}

void ProgarmErrorHanlder(GLuint program_id) {
  GLint log_size;
  glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &log_size);
  log_size = log_size > 0 ? log_size : 1024;
  std::vector<char> errorMessage(log_size);
  glGetProgramInfoLog(program_id, log_size, nullptr, errorMessage.data());
  std::cerr << "Linking Error: " << errorMessage.data() << std::endl;
}

GLuint CompileShaderFromSource(const char *data, GLenum type) {
  GLuint shader_id = glCreateShader(type);
  glShaderSource(shader_id, 1, &data, nullptr);
  glCompileShader(shader_id);
  GLint result = GL_FALSE;
  glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
  if (result != GL_TRUE) {
    ShaderErrorHandler(shader_id);
    return 0;
  }
  return shader_id;
}

GLuint CompileShaderFromFile(const char *fname, GLenum type) {
  std::string shader_text = diagrammar::Stringify(fname);
  const char *shader_text_cstr = shader_text.c_str();
  return CompileShaderFromSource(shader_text_cstr, type);
}

GLuint CreateGLProgram(const char *vert_shader_src,
                       const char *frag_shader_src) {
  GLuint vert_shader_id =
      CompileShaderFromSource(vert_shader_src, GL_VERTEX_SHADER);
  GLuint frag_shader_id =
      CompileShaderFromSource(frag_shader_src, GL_FRAGMENT_SHADER);
  // now we can link the program
  GLuint program_id = glCreateProgram();
  glAttachShader(program_id, vert_shader_id);
  glAttachShader(program_id, frag_shader_id);
  glLinkProgram(program_id);
  GLint result;
  glGetProgramiv(program_id, GL_LINK_STATUS, &result);
  if (result != GL_TRUE) {
    ProgarmErrorHanlder(program_id);
  }
  glDeleteShader(vert_shader_id);
  glDeleteShader(frag_shader_id);

  return program_id;
}

std::vector<GLfloat> SerializePath2D(const Path2D &path, bool is_closed) {
  std::vector<GLfloat> vertices(path.size() * kDiagrammarGLVertexDimension);
  const size_t path_size = path.size();
  for (size_t i = 0, j = 0; i < path_size; ++i, j += 4) {
    vertices[j + 0] = path[i](0);
    vertices[j + 1] = path[i](1);
    vertices[j + 2] = 0;
    vertices[j + 3] = 1;
  }
  if (is_closed) {
    vertices.emplace_back(path[0](0));
    vertices.emplace_back(path[0](1));
    vertices.emplace_back(0);
    vertices.emplace_back(0);
  }
  return vertices;
}

GLTriangleMesh::GLTriangleMesh(GLuint num_vertices, GLuint num_triangles)
    : vertices(kDiagrammarGLVertexDimension * num_vertices),
      normals(kDiagrammarGLVertexDimension * num_vertices),
      colors(kDiagrammarGLVertexDimension * num_vertices),
      indices(3 * num_triangles) {}

GLTriangleMesh::GLTriangleMesh(GLuint num_vertices)
    : vertices(kDiagrammarGLVertexDimension * num_vertices),
      normals(kDiagrammarGLVertexDimension * num_vertices),
      colors(kDiagrammarGLVertexDimension * num_vertices) {}

GLTriangleMesh ConvertDiagMesh2DToGLMesh(const TriangleMesh2D &diag_mesh,
                                         GLfloat depth, bool normal_up) {
  int normal_sign = normal_up ? 1 : -1;
  GLTriangleMesh gl_mesh(diag_mesh.vertices.size(), diag_mesh.faces.size());
  const size_t num_vertices = diag_mesh.vertices.size();
  for (size_t i = 0, j = 0; i < num_vertices;
       ++i, j += kDiagrammarGLVertexDimension) {
    gl_mesh.vertices[j + 0] = diag_mesh.vertices[i](0);
    gl_mesh.vertices[j + 1] = diag_mesh.vertices[i](1);
    gl_mesh.vertices[j + 2] = depth;
    gl_mesh.vertices[j + 3] = 1;

    gl_mesh.normals[j + 0] = 0;
    gl_mesh.normals[j + 1] = 0;
    gl_mesh.normals[j + 2] = normal_sign;
    gl_mesh.normals[j + 3] = 1;

    if (i % 3 == 0) {
      // r
      gl_mesh.colors[j + 0] = 1;
      gl_mesh.colors[j + 1] = 0;
      gl_mesh.colors[j + 2] = 0;
      gl_mesh.colors[j + 3] = 1;
    }

    if (i % 3 == 1) {
      // g
      gl_mesh.colors[j + 0] = 0;
      gl_mesh.colors[j + 1] = 1;
      gl_mesh.colors[j + 2] = 0;
      gl_mesh.colors[j + 3] = 1;
    }

    if (i % 3 == 2) {
      // b
      gl_mesh.colors[j + 0] = 0;
      gl_mesh.colors[j + 1] = 0;
      gl_mesh.colors[j + 2] = 1;
      gl_mesh.colors[j + 3] = 1;
    }
  }

  const size_t num_faces = diag_mesh.faces.size();
  for (size_t i = 0, j = 0; i < num_faces; ++i, j += 3) {
    gl_mesh.indices[j + 0] = diag_mesh.faces[i].at(0);
    gl_mesh.indices[j + 1] = diag_mesh.faces[i].at(1);
    gl_mesh.indices[j + 2] = diag_mesh.faces[i].at(2);
  }

  return gl_mesh;
}

GLTriangleMesh SweeptPath2DToGLMesh(const Path2D &path, GLfloat depth,
                                    bool is_closed, bool outward) {
  size_t num_path_points = path.size();
  // Two set of vertices for top and bottom.
  size_t num_vertices = 2 * num_path_points;
  size_t num_triangles = is_closed ? num_vertices : num_vertices - 2;
  GLTriangleMesh gl_mesh(num_vertices, num_triangles);
  auto path_normals = GeneratePathNormals(path, is_closed, outward);
  for (size_t i = 0, j = 0; i < num_path_points; ++i, j += 8) {
    // Bottom vertices
    gl_mesh.vertices[j + 0] = path[i](0);
    gl_mesh.vertices[j + 1] = path[i](1);
    gl_mesh.vertices[j + 2] = -depth * 0.5;
    gl_mesh.vertices[j + 3] = 1;
    // Top vertices
    gl_mesh.vertices[j + 4] = path[i](0);
    gl_mesh.vertices[j + 5] = path[i](1);
    gl_mesh.vertices[j + 6] = depth * 0.5;
    gl_mesh.vertices[j + 7] = 1;
    // Bottom normal
    gl_mesh.normals[j + 0] = path_normals[i](0);
    gl_mesh.normals[j + 1] = path_normals[i](1);
    gl_mesh.normals[j + 2] = 0;
    gl_mesh.normals[j + 3] = 1;
    // Top normal
    gl_mesh.normals[j + 4] = path_normals[i](0);
    gl_mesh.normals[j + 5] = path_normals[i](1);
    gl_mesh.normals[j + 6] = 0;
    gl_mesh.normals[j + 7] = 1;
    // Colors
    for (size_t k = 0; k < 8; ++k) {
      gl_mesh.colors[j + k] = 1;
    }
  }
  // We have triangles connecting the top and bottom vertices.
  // The vertices are grouped as [0 1 2, 1 2 3, 2 3 4, 3 4 5...]
  size_t i = 0, j = 0;
  for (; i < num_vertices - 2; i += 2, j += 6) {
    gl_mesh.indices[j + 0] = i;
    gl_mesh.indices[j + 1] = i + 1;
    gl_mesh.indices[j + 2] = i + 2;

    gl_mesh.indices[j + 3] = i + 1;
    gl_mesh.indices[j + 4] = i + 2;
    gl_mesh.indices[j + 5] = i + 3;
  }

  if (is_closed) {
    assert(i + 2 == num_vertices);
    gl_mesh.indices[j + 0] = i;
    gl_mesh.indices[j + 1] = i + 1;
    gl_mesh.indices[j + 2] = 0;

    gl_mesh.indices[j + 3] = i + 1;
    gl_mesh.indices[j + 4] = 0;
    gl_mesh.indices[j + 5] = 1;
  }
  return gl_mesh;
}

GLTriangleMesh SweepPolygon2DToGLMesh(const Polygon2D &polygon, GLfloat depth) {
  std::vector<GLTriangleMesh> gl_meshes;
  gl_meshes.emplace_back(SweeptPath2DToGLMesh(polygon.path, depth));
  for (const auto &hole : polygon.holes) {
    gl_meshes.emplace_back(SweeptPath2DToGLMesh(hole, depth, true, false));
  }
  auto diag_mesh = TriangulatePolygon(polygon);
  gl_meshes.emplace_back(
      ConvertDiagMesh2DToGLMesh(diag_mesh, -0.5 * depth, false));
  gl_meshes.emplace_back(
      ConvertDiagMesh2DToGLMesh(diag_mesh, depth * 0.5, true));
  return CombineGLMesh(gl_meshes);
}

GLTriangleMesh CombineGLMesh(std::vector<GLTriangleMesh> meshes) {
  GLTriangleMesh combined_mesh;
  // The only thing we need to worry about is the indices array;
  for (auto &mesh : meshes) {
    // We MUST insert at the end, to correctly use index offset
    size_t offset = combined_mesh.vertices.size() / 4;
    combined_mesh.vertices.insert(
        combined_mesh.vertices.end(),
        std::make_move_iterator(mesh.vertices.begin()),
        std::make_move_iterator(mesh.vertices.end()));
    combined_mesh.normals.insert(combined_mesh.normals.end(),
                                 std::make_move_iterator(mesh.normals.begin()),
                                 std::make_move_iterator(mesh.normals.end()));
    combined_mesh.colors.insert(combined_mesh.colors.end(),
                                std::make_move_iterator(mesh.colors.begin()),
                                std::make_move_iterator(mesh.colors.end()));
    // Update the current mesh indices before inserting to the back of combined.
    for (auto &idx : mesh.indices) {
      idx += offset;
    }
    combined_mesh.indices.insert(combined_mesh.indices.end(),
                                 std::make_move_iterator(mesh.indices.begin()),
                                 std::make_move_iterator(mesh.indices.end()));
  }
  return combined_mesh;
}

GLTriangleMesh GLTriangulate2DShape2D(const CollisionShape2D *shape_ptr) {
  switch (shape_ptr->shape_type) {
    case Shape2DType::kDisk: {
      auto disk_ptr = dynamic_cast<const Disk2D *>(shape_ptr);
      const size_t num_vertices = 30;
      Polygon2D polygon(SketchDiskEdge(*disk_ptr, num_vertices));
      auto diag_mesh = TriangulatePolygon(polygon);
      return ConvertDiagMesh2DToGLMesh(diag_mesh, 0.0);
    } break;
    case Shape2DType::kPolygon: {
      auto poly_ptr = dynamic_cast<const Polygon2D *>(shape_ptr);
      auto diag_mesh = TriangulatePolygon(*poly_ptr);
      return ConvertDiagMesh2DToGLMesh(diag_mesh, 0.0);
    } break;
    case Shape2DType::kPolyLine: {
      auto line_ptr = dynamic_cast<const Line2D *>(shape_ptr);
      auto diag_mesh = TriangulatePolyline(line_ptr->path, 1.5);
      return ConvertDiagMesh2DToGLMesh(diag_mesh, 0.0);
    } break;
    default:
      break;
  }
  return GLTriangleMesh();
}

GLTriangleMesh GLTriangulate3DShape2D(const CollisionShape2D *shape_ptr,
                                      GLfloat depth) {
  switch (shape_ptr->shape_type) {
    case Shape2DType::kDisk: {
      auto disk_ptr = dynamic_cast<const Disk2D *>(shape_ptr);
      const size_t num_vertices = 30;
      Polygon2D polygon(SketchDiskEdge(*disk_ptr, num_vertices));
      return SweepPolygon2DToGLMesh(polygon, depth);
    } break;
    case Shape2DType::kPolygon: {
      auto poly_ptr = dynamic_cast<const Polygon2D *>(shape_ptr);
      return SweepPolygon2DToGLMesh(*poly_ptr, depth);
    } break;
    case Shape2DType::kPolyLine: {
      auto line_ptr = dynamic_cast<const Line2D *>(shape_ptr);
      return SweeptPath2DToGLMesh(line_ptr->path, depth, false);
    } break;
    default:
      break;
  }
  return GLTriangleMesh();
}

}  // namespace diagrammar
