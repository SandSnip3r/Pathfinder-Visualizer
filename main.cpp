#include "main_window.hpp"

#include <QApplication>

#include <absl/flags/parse.h>
#include <absl/log/log_sink.h>
#include <absl/log/log_sink_registry.h>

#include <iostream>
#include <chrono>
#include <thread>


int main(int argc, char *argv[]) {
  // Initialize abseil logging.
  absl::InitializeLog();
  // Ideally, this setting of the threshold and vlog level should come from the commandline, but something about our cmake setup results in the relevant flags not being linked into this binary.
  absl::SetGlobalVLogLevel(10);

  // QtCreator has a bug where every line of output from Abseil log statements is printed twice in the Application Output window.
  //  Bug: https://bugreports.qt.io/browse/QTCREATORBUG-30163
  constexpr bool kWorkaroundForQtCreatorBug{true};
  if constexpr (kWorkaroundForQtCreatorBug) {
    // To work around this bug, don't use the built-in logging mechanism, instead create our own LogSink which does the outputting for us.
    class MyLogSink : public absl::LogSink {
    public:
      void Send(const absl::LogEntry& entry) override {
        std::cout << entry.text_message_with_prefix_and_newline() << std::flush;
      }
    };
    absl::LogSink *mySink = new MyLogSink;
    absl::AddLogSink(mySink);
    absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfinity);
  } else {
    absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  }

  QApplication a(argc, argv);
  MainWindow w;
  w.move(0,0);
  w.show();
  return a.exec();
}
