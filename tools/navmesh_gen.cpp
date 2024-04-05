#include <algorithm>
#include <array>
#include <functional>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace std;

mt19937 createRandomEngine() {
  random_device rd;
  array<int, mt19937::state_size> seed_data;
  generate_n(seed_data.data(), seed_data.size(), ref(rd));
  seed_seq seq(begin(seed_data), end(seed_data));
  return mt19937(seq);
}

class Navmesh {
public:
  explicit Navmesh(double minX, double minY, double width, double height) {
    // Define corners
    const auto v0 = addVertex(0, 0);
    const auto v1 = addVertex(width, 0);
    const auto v2 = addVertex(0, height);
    const auto v3 = addVertex(width, height);
    // Define bounding box edges
    addEdge(v0, v1);
    addEdge(v0, v2);
    addEdge(v1, v3);
    addEdge(v2, v3);
  }

  int addVertex(double x, double y) {
    // Check if this vertex exists.
    const auto it = find_if(vertices_.begin(), vertices_.end(), [&x, &y](const auto &v) {
      return v.x == x && v.y == y;
    });
    if (it != vertices_.end()) {
      // Vertex already exists.
      return std::distance(vertices_.begin(), it);
    }

    // Vertex does not exist.
    vertices_.emplace_back(x, y);
    return vertices_.size() - 1;
  }

  void addEdge(int v0, int v1) {
    edges_.emplace_back(v0, v1);
  }

  void print() const {
    // Print <num vertices> 2 0 0
    cout << vertices_.size() << " 2 0 0" << endl;
    // Print each vertex as
    //  <index> <x> <y>
    for (int i=0; i<vertices_.size(); ++i) {
      cout << i << ' ' << fixed << setprecision(12) << vertices_.at(i).x << ' ' << vertices_.at(i).y << endl;
    }

    // Print <num edges> 1
    cout << edges_.size() << " 1" << endl;
    // Print each edge as
    //  <index> <start vertex index> <end vertex index> 3
    for (int i=0; i<edges_.size(); ++i) {
      cout << i << ' ' << edges_.at(i).startVertexIndex << ' ' << edges_.at(i).endVertexIndex << " 3" << endl;
    }

    // Print number of holes
    cout << 0 << endl;
  }

private:
  struct Vertex {
    Vertex(double x, double y) : x(x), y(y) {}
    double x, y;
  };
  struct Edge {
    Edge(int startVertexIndex, int endVertexIndex) : startVertexIndex(startVertexIndex), endVertexIndex(endVertexIndex) {}
    int startVertexIndex, endVertexIndex;
  };
  std::vector<Vertex> vertices_;
  std::vector<Edge> edges_;
};

int main() {
  constexpr double kWidth = 1000;
  constexpr double kHeight = 1000;
  Navmesh navmesh(0.0, 0.0, kWidth, kHeight);

  auto eng = createRandomEngine();
  // uniform_real_distribution<double> xDist(0.0, kWidth);
  // uniform_real_distribution<double> yDist(0.0, kHeight);
  normal_distribution<double> xDist(kWidth/2.0, 100);
  normal_distribution<double> yDist(kHeight/2.0, 100);
  constexpr int kObjectCount = 250;
  constexpr double kSize = 1.0;
  for (int i=0; i<kObjectCount; ++i) {
    double x,y;
    do {
      x = xDist(eng);
      y = yDist(eng);
    } while (x >= kWidth * 0.45 && x <= kWidth * 0.55 &&
             y >= kHeight * 0.45 && y <= kHeight * 0.55);
    auto v0 = navmesh.addVertex(x - kSize/2.0, y - kSize/2.0);
    auto v1 = navmesh.addVertex(x + kSize/2.0, y - kSize/2.0);
    auto v2 = navmesh.addVertex(x - kSize/2.0, y + kSize/2.0);
    auto v3 = navmesh.addVertex(x + kSize/2.0, y + kSize/2.0);
    navmesh.addEdge(v0, v1);
    navmesh.addEdge(v0, v2);
    navmesh.addEdge(v1, v3);
    navmesh.addEdge(v2, v3);
    // navmesh.addEdge(v0, v3);
    // navmesh.addEdge(v1, v2);
  }
  navmesh.print();
  return 0;
}