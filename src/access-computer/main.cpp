#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "planner/run_planner.hpp"
#include "planner/validate.hpp"
#include "satellite/exit_codes.hpp"
#include "satellite/json_io.hpp"

namespace {

constexpr const char* kVersion = "0.1.0";

struct CliOptions {
    std::string                          command;
    std::optional<std::filesystem::path> input_path;
    std::optional<std::filesystem::path> work_dir;
    bool                                 use_stdin = false;
    bool                                 dry_run   = false;
    bool                                 pretty    = false;
    bool                                 show_help = false;
};

void print_global_usage() {
    std::cerr << "access-computer " << kVersion
              << " - agent-friendly access windows compute CLI plugin\n\n"
              << "Usage:\n"
              << "  access-computer manifest [--output json|json-pretty]\n"
              << "  access-computer validate (--input FILE | --stdin) "
                 "[--output json|json-pretty]\n"
              << "  access-computer run (--input FILE | --stdin) --work-dir "
                 "DIR [--dry-run] [--output json|json-pretty]\n\n"
              << "Global options:\n"
              << "  --help, -h       Show help (global or per subcommand)\n"
              << "  --version        Print version and exit\n\n"
              << "Examples:\n"
              << "  access-computer manifest --output json\n"
              << "  access-computer validate --input "
                 "samples/remote_sensing_access.json\n"
              << "  access-computer run --input "
                 "samples/remote_sensing_access.json --work-dir /tmp/mp_task\n"
              << "  cat samples/remote_sensing_access.json | access-computer "
                 "run --stdin --work-dir /tmp/mp_task --dry-run\n";
}

void print_manifest_usage() {
    std::cerr << "access-computer manifest - print plugin manifest JSON\n\n"
              << "Usage:\n"
              << "  access-computer manifest [--output json|json-pretty]\n\n"
              << "Examples:\n"
              << "  access-computer manifest --output json\n"
              << "  access-computer manifest --output json-pretty\n";
}

void print_validate_usage() {
    std::cerr
        << "access-computer validate - validate a planning request without "
           "running GMAT\n\n"
        << "Usage:\n"
        << "  access-computer validate --input request.json [--output "
           "json|json-pretty]\n"
        << "  access-computer validate --stdin [--output json|json-pretty]\n\n"
        << "Examples:\n"
        << "  access-computer validate --input "
           "samples/remote_sensing_access.json\n"
        << "  access-computer validate --input samples/sar_unsupported.json\n"
        << "  cat samples/attitude_estimation.json | access-computer validate "
           "--stdin\n";
}

void print_run_usage() {
    std::cerr
        << "access-computer run - execute access windows compute (GMAT "
           "backend)\n\n"
        << "Usage:\n"
        << "  access-computer run --input request.json --work-dir DIR "
           "[--dry-run] [--output json|json-pretty]\n"
        << "  access-computer run --stdin --work-dir DIR [--dry-run] [--output "
           "json|json-pretty]\n\n"
        << "Options:\n"
        << "  --work-dir DIR   Writable task directory (required)\n"
        << "  --dry-run        Validate and return plan metadata without "
           "invoking GMAT\n"
        << "  --output         json (default) or json-pretty\n\n"
        << "Examples:\n"
        << "  access-computer run --input samples/remote_sensing_access.json "
           "--work-dir /tmp/mp_task\n"
        << "  access-computer run --input samples/remote_sensing_access.json "
           "--work-dir /tmp/mp_task --dry-run\n"
        << "  cat samples/downlink_window.json | access-computer run --stdin "
           "--work-dir /tmp/mp_downlink\n";
}

void print_command_help(const std::string& command) {
    if (command == "manifest") { print_manifest_usage(); }
    else if (command == "validate") { print_validate_usage(); }
    else if (command == "run") { print_run_usage(); }
    else { print_global_usage(); }
}

bool parse_args(int argc, char** argv, CliOptions& opts) {
    if (argc < 2) { return false; }
    opts.command = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) { opts.input_path = argv[++i]; }
        else if (arg == "--work-dir" && i + 1 < argc) {
            opts.work_dir = argv[++i];
        }
        else if (arg == "--stdin") { opts.use_stdin = true; }
        else if (arg == "--dry-run") { opts.dry_run = true; }
        else if (arg == "--output" && i + 1 < argc) {
            const std::string fmt = argv[++i];
            opts.pretty           = (fmt == "json-pretty");
        }
        else if (arg == "--help" || arg == "-h") { opts.show_help = true; }
        else {
            std::cerr << "Unknown argument: " << arg << '\n';
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2) {
        const std::string first = argv[1];
        if (first == "--version") {
            std::cout << "access-computer " << kVersion << '\n';
            return satellite::EXIT_OK;
        }
        if (first == "--help" || first == "-h") {
            print_global_usage();
            return satellite::EXIT_OK;
        }
    }

    CliOptions opts;
    if (!parse_args(argc, argv, opts)) {
        print_global_usage();
        return satellite::EXIT_USAGE;
    }

    if (opts.show_help) {
        print_command_help(opts.command);
        return satellite::EXIT_OK;
    }

    try {
        if (opts.command == "manifest") {
            satellite::write_json_stdout(mp::make_manifest(), opts.pretty);
            return satellite::EXIT_OK;
        }

        if (opts.command == "validate") {
            if (!opts.input_path && !opts.use_stdin) {
                std::cerr << "Error: --input request.json or --stdin is "
                             "required for validate\n";
                std::cerr << "  access-computer validate --input "
                             "samples/remote_sensing_access.json\n";
                std::cerr << "  cat request.json | access-computer validate "
                             "--stdin\n";
                return satellite::EXIT_VALIDATION;
            }
            const auto request =
                satellite::read_json_input(opts.input_path, opts.use_stdin);
            const auto result = mp::validate_request(request);
            satellite::write_json_stdout(
                {
                    {"ok", result.ok},
                    {"message", result.message},
                    {"details", result.details},
                },
                opts.pretty);
            return result.ok ? satellite::EXIT_OK : satellite::EXIT_VALIDATION;
        }

        if (opts.command == "run") {
            if (!opts.work_dir) {
                std::cerr << "Error: --work-dir is required for run\n";
                std::cerr << "  access-computer run --input request.json "
                             "--work-dir /tmp/task_xxx\n";
                std::cerr << "  access-computer run --stdin --work-dir "
                             "/tmp/task_xxx --dry-run\n";
                return satellite::EXIT_VALIDATION;
            }
            if (!opts.input_path && !opts.use_stdin) {
                std::cerr << "Error: --input request.json or --stdin is "
                             "required for run\n";
                std::cerr << "  access-computer run --input request.json "
                             "--work-dir /tmp/task_xxx\n";
                std::cerr << "  cat request.json | access-computer run --stdin "
                             "--work-dir /tmp/task_xxx\n";
                return satellite::EXIT_VALIDATION;
            }
            const auto request =
                satellite::read_json_input(opts.input_path, opts.use_stdin);
            mp::RunContext ctx;
            ctx.work_dir = *opts.work_dir;
            ctx.dry_run  = opts.dry_run;
            if (request.contains("request_id")) {
                ctx.task_id = request.at("request_id").get<std::string>();
            }
            if (request.contains("trace_id")) {
                ctx.trace_id = request.at("trace_id").get<std::string>();
            }
            const auto output = mp::run_planner(request, ctx);
            satellite::write_json_stdout(output, opts.pretty);
            if (output.value("status", "") == "no_result") {
                return satellite::EXIT_NO_RESULT;
            }
            return satellite::EXIT_OK;
        }

        std::cerr << "Unknown command: " << opts.command << '\n';
        print_global_usage();
        return satellite::EXIT_USAGE;
    } catch (const std::exception& ex) {
        satellite::write_json_stdout(
            {
                {"ok", false},
                {"error", ex.what()},
            },
            false);
        const std::string msg = ex.what();
        if (msg.find("not found") != std::string::npos) {
            return satellite::EXIT_DEPENDENCY;
        }
        if (msg.find("validation") != std::string::npos ||
            msg.find("Missing") != std::string::npos) {
            return satellite::EXIT_VALIDATION;
        }
        return satellite::EXIT_RETRYABLE;
    }
}
