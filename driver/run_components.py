# -*- coding: utf-8 -*-

import logging
import os
import os.path
import subprocess
import sys

from . import call
from . import util

REL_TRANSLATE_PATH = os.path.join("translate", "translate.py")
if os.name == "posix":
    REL_PREPROCESS_PATH = "preprocess"
    REL_SEARCH_PATH = "downward"
    VALIDATE = "validate"
elif os.name == "nt":
    REL_PREPROCESS_PATH = "preprocess.exe"
    REL_SEARCH_PATH = "downward.exe"
    VALIDATE = "validate.exe"
else:
    sys.exit("Unsupported OS: " + os.name)

def get_executable(build, rel_path):
    # First, consider 'build' to be a path directly to the binaries.
    # The path can be absolute or relative to the current working
    # directory.
    build_dir = build
    if not os.path.exists(build_dir):
        # If build is not a full path to the binaries, it might be the
        # name of a build in our standard directory structure.
        # in this case, the binaries are in
        #   '<repo-root>/builds/<buildname>/bin'.
        build_dir = os.path.join(util.BUILDS_DIR, build, "bin")
    if not os.path.exists(build_dir) and os.name == "nt":
        # If the build is still not found, try to detect a manual
        # CMake build for Visual Studio.
        if build.startswith("debug"):
            build_dir = os.path.join(util.BUILDS_DIR, "bin", "Debug")
        elif build.startswith("release"):
            build_dir = os.path.join(util.BUILDS_DIR, "bin", "Release")
    if not os.path.exists(build_dir):
        raise IOError(
            "Could not find build '{build}' at {build_dir}. "
            "Please run './build.py {build}'.".format(**locals()))

    abs_path = os.path.join(build_dir, rel_path)
    if not os.path.exists(abs_path):
        raise IOError(
            "Could not find '{rel_path}' in build '{build}'. "
            "Please run './build.py {build}'.".format(**locals()))

    return abs_path

def print_callstring(executable, options, stdin):
    parts = [executable] + options
    parts = [util.shell_escape(x) for x in parts]
    if stdin is not None:
        parts.extend(["<", util.shell_escape(stdin)])
    logging.info("callstring: %s" % " ".join(parts))

def call_component(executable, options, stdin=None):
    if executable.endswith(".py"):
        options.insert(0, executable)
        executable = sys.executable
        assert executable, "Path to interpreter could not be found"
    print_callstring(executable, options, stdin)
    call.check_call([executable] + options, stdin=stdin)

def run_translate(args):
    logging.info("Running translator.")
    translate = get_executable(args.build, REL_TRANSLATE_PATH)
    call_component(translate, args.translate_inputs + args.translate_options)

def run_preprocess(args):
    logging.info("Running preprocessor (%s)." % args.build)
    preprocess = get_executable(args.build, REL_PREPROCESS_PATH)
    call_component(preprocess, args.preprocess_options, stdin=args.preprocess_input)

def run_search(args):
    logging.info("Running search (%s)." % args.build)

    search = get_executable(args.build, REL_SEARCH_PATH)
    logging.info("search executable: %s" % search)

    if not args.search_options:
        raise ValueError(
            "search needs --alias, --portfolio, or search options")
    if "--help" not in args.search_options:
        args.search_options.extend(["--plan-file", args.plan_file])
    try:
        call_component(
            search, args.search_options,
            stdin=args.search_input)
    except subprocess.CalledProcessError as err:
        return err.returncode
    else:
        return 0

