// ============================================================
// trajectory_trail — постоянный трек траектории дрона в Gazebo.
//
// Подписывается на топик PosePublisher модели (по умолчанию
// /model/x500_depth/pose, gz.msgs.Pose_V; имя дрона в поле name) и
// публикует накопленную траекторию в виде LINE_STRIP + POINTS маркеров.
// Трек накапливается в течение всей миссии и не стирается.
//
// ВАЖНО (gz-sim8 / Harmonic): маркеры НЕ отрисовываются из топика /marker —
// ни серверный rendering MarkerManager, ни GUI-плагин MarkerManager
// (gz-gui) не подписываются на топик. Единственный способ добавить маркеры
// в сцену — ВЫЗОВ СЕРВИСА:
//   * /marker, /marker_array  — GUI-сцена (видна в gz sim -g);
//   * /sensors/marker, /sensors/marker_array — сцена сенсоров (камеры).
// Поэтому трек публикуется и в топик /marker (для внешних инструментов),
// и отправляется в сервис /marker_array (асинхронно, с троттлингом).
//
// Сборка:
//     bash tools/trajectory_trail/build.sh
//
// Запуск:
//     ./trajectory_trail --pose-topic /model/x500_depth/pose --model x500
// ============================================================
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

class TrajectoryTrail
{
public:
  TrajectoryTrail(const std::string &_poseTopic,
                  const std::string &_modelPrefix,
                  double _sampleDistance)
    : modelPrefix(_modelPrefix), sampleDistance(_sampleDistance)
  {
    if (!node.Subscribe(_poseTopic, &TrajectoryTrail::OnPoseV, this))
    {
      std::cerr << "[trail] не удалось подписаться на " << _poseTopic
                << std::endl;
    }
    markerPub = node.Advertise<gz::msgs::Marker>("/marker");
    std::cout << "[trail] подписка: " << _poseTopic
              << " | маркер: /marker" << std::endl;
  }

  void OnPoseV(const gz::msgs::Pose_V &_msg)
  {
    std::lock_guard<std::mutex> lock(mtx);
    for (int i = 0; i < _msg.pose_size(); ++i)
    {
      const auto &pose = _msg.pose(i);
      if (pose.name().find(modelPrefix) == std::string::npos)
        continue;
      gz::math::Vector3d p(
          pose.position().x(), pose.position().y(), pose.position().z());
      if (points.empty())
      {
        points.push_back(p);
        std::cout << "[trail] старт трека: " << p << std::endl;
      }
      else if (points.back().Distance(p) >= sampleDistance)
      {
        points.push_back(p);
      }
    }
  }

  void Publish()
  {
    std::vector<gz::math::Vector3d> pts;
    {
      std::lock_guard<std::mutex> lock(mtx);
      pts = points;
    }
    if (pts.size() < 2)
      return;

    // Линия (LINE_STRIP).
    gz::msgs::Marker line;
    line.set_ns("trajectory_trail");
    line.set_id(1);
    line.set_action(gz::msgs::Marker::ADD_MODIFY);
    line.set_type(gz::msgs::Marker::LINE_STRIP);
    line.set_layer(0);
    line.set_visibility(gz::msgs::Marker::GUI);
    line.mutable_lifetime()->set_sec(0);
    line.mutable_lifetime()->set_nsec(0);
    for (const auto &p : pts)
    {
      auto *pt = line.add_point();
      pt->set_x(p.X());
      pt->set_y(p.Y());
      pt->set_z(p.Z());
    }
    auto *lmat = line.mutable_material();
    lmat->mutable_emissive()->set_r(1.0f);
    lmat->mutable_emissive()->set_g(0.35f);
    lmat->mutable_emissive()->set_b(0.0f);
    lmat->mutable_emissive()->set_a(1.0f);
    lmat->mutable_diffuse()->set_r(1.0f);
    lmat->mutable_diffuse()->set_g(0.35f);
    lmat->mutable_diffuse()->set_b(0.0f);
    lmat->mutable_diffuse()->set_a(1.0f);
    markerPub.Publish(line);

    // Точки (POINTS) — делают трек заметнее.
    gz::msgs::Marker dots;
    dots.set_ns("trajectory_trail_points");
    dots.set_id(2);
    dots.set_action(gz::msgs::Marker::ADD_MODIFY);
    dots.set_type(gz::msgs::Marker::POINTS);
    dots.set_layer(0);
    dots.set_visibility(gz::msgs::Marker::GUI);
    dots.mutable_lifetime()->set_sec(0);
    dots.mutable_lifetime()->set_nsec(0);
    dots.mutable_scale()->set_x(0.12);
    dots.mutable_scale()->set_y(0.12);
    dots.mutable_scale()->set_z(0.12);
    for (const auto &p : pts)
    {
      auto *pt = dots.add_point();
      pt->set_x(p.X());
      pt->set_y(p.Y());
      pt->set_z(p.Z());
    }
    auto *dmat = dots.mutable_material();
    dmat->mutable_emissive()->set_r(1.0f);
    dmat->mutable_emissive()->set_g(0.35f);
    dmat->mutable_emissive()->set_b(0.0f);
    dmat->mutable_emissive()->set_a(1.0f);
    dmat->mutable_diffuse()->set_r(1.0f);
    dmat->mutable_diffuse()->set_g(0.35f);
    dmat->mutable_diffuse()->set_b(0.0f);
    dmat->mutable_diffuse()->set_a(1.0f);
    markerPub.Publish(dots);

    // Отправляем маркеры в сервис /marker_array (gz-gui MarkerManager) —
    // единственный путь, по которому gz-sim8 реально рисует маркеры.
    // Асинхронно и не чаще, чем при добавлении новых точек / каждую 1 с:
    // вызов возвращается мгновенно, даже если GUI сейчас не запущен.
    auto now = std::chrono::steady_clock::now();
    if (pts.size() != lastServicePointCount ||
        now - lastServiceSend > std::chrono::milliseconds(1000))
    {
      SendMarkersViaService(line, dots);
      lastServicePointCount = pts.size();
      lastServiceSend = now;
    }
  }

  // static callback for service response
  static void ServiceCallback(const gz::msgs::Boolean &rep, bool ok)
  {
    if (!ok || !rep.data())
      std::cerr << "[trail] SRV cb: ok=" << ok << " data=" << rep.data() << std::endl;
  }

  // Асинхронный вызов /marker_array (или /marker) с обоими маркерами.
  // fire-and-forget: не блокирует цикл; при отсутствии GUI запрос просто
  // не находит сервис и ничего не делает.
  void SendMarkersViaService(const gz::msgs::Marker &_line,
                             const gz::msgs::Marker &_dots)
  {
    gz::msgs::Marker_V v;
    *v.add_marker() = _line;
    *v.add_marker() = _dots;
    node.Request("/marker_array", v, &ServiceCallback);
  }

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher markerPub;
  std::string modelPrefix;
  double sampleDistance;
  std::vector<gz::math::Vector3d> points;
  std::mutex mtx;
  std::size_t lastServicePointCount = 0;
  std::chrono::steady_clock::time_point lastServiceSend{};
};

}  // namespace

int main(int argc, char **argv)
{
  std::string poseTopic = "/model/x500_depth/pose";
  std::string model = "x500";
  double sampleDistance = 0.05;
  double rate = 20.0;

  for (int i = 1; i < argc; ++i)
  {
    std::string a = argv[i];
    if (a == "--pose-topic" && i + 1 < argc)
      poseTopic = argv[++i];
    else if (a == "--model" && i + 1 < argc)
      model = argv[++i];
    else if (a == "--sample-distance" && i + 1 < argc)
      sampleDistance = std::stod(argv[++i]);
    else if (a == "--rate" && i + 1 < argc)
      rate = std::stod(argv[++i]);
    else if (a == "--help" || a == "-h")
    {
      std::cout << "usage: trajectory_trail [--pose-topic TOPIC] "
                   "[--model PREFIX] [--sample-distance M] [--rate HZ]\n";
      return 0;
    }
    else
    {
      std::cerr << "[trail] неизвестный аргумент: " << a << std::endl;
      return 1;
    }
  }

  std::cout << "[trail] topic=" << poseTopic << " model=" << model
            << " sample=" << sampleDistance << " rate=" << rate << std::endl;

  TrajectoryTrail trail(poseTopic, model, sampleDistance);

  const auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / rate));
  while (true)
  {
    trail.Publish();
    std::this_thread::sleep_for(period);
  }
  return 0;
}
