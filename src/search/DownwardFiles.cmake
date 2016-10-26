set(PLANNER_SOURCES
  planner.cc
  )

# See http://www.fast-downward.org/ForDevelopers/AddingSourceFiles
# for general information on adding source files and CMake plugins.
#
# If you're adding a file to the codebase which *isn't* a plugin, add
# it to the following list. We assume that every *.cc file has a
# corresponding *.h file and add headers to the project automatically.
# For plugin files, see below.

set(CORE_SOURCES
  axioms.cc
  causal_graph.cc
  domain_transition_graph.cc
  globals.cc
  heuristic.cc
  int_packer.cc
  memory.cc
  operator.cc
  operator_cost.cc
  option_parser.cc
  option_parser_util.cc
  segmented_vector.cc
  per_state_information.cc
  rng.cc
  search_node_info.cc
  search_space.cc
  search_engine.cc
  search_progress.cc
  state.cc
  state_id.cc
  state_registry.cc
  successor_generator.cc
  timer.cc
  utilities.cc
  pruning_method.cc
  plugin.h
  evaluator.h
  scalar_evaluator.h
  doc.h
  priority_queue.h
  open_list.h
  language.h
  )

fast_downward_add_headers_to_sources_list(CORE_SOURCES)
source_group(core FILES planner.cc ${CORE_SOURCES})
list(APPEND PLANNER_SOURCES ${CORE_SOURCES})

## Details of the plugins
#
# For now, everything defaults to being enabled - it's up to the user to specify
#    -DPLUGIN_FOO_ENABLED=FALSE
# to disable a given plugin.
#
# Defining a new plugin:
#    fast_downward_plugin(
#        NAME <NAME>
#        [ DISPLAY_NAME <DISPLAY_NAME> ]
#        [ HELP <HELP> ]
#        SOURCES
#            <FILE_1> [ <FILE_2> ... ]
#        [ DEPENDS <PLUGIN_NAME_1> [ <PLUGIN_NAME_2> ... ] ]
#        [ DEPENDENCY_ONLY ]
#        [ CORE_PLUGIN ]
#    )
#
# <DISPLAY_NAME> defaults to lower case <NAME> and is used to group
#                files in IDEs and for messages.
# <HELP> defaults to <DISPLAY_NAME> and is used to describe the cmake option.
# DEPENDS lists plugins that will be automatically enabled if this plugin
# is enabled. If the dependency was not enabled before, this will be logged.
# DEPENDENCY_ONLY disables the plugin unless it is needed as a dependency and
#     hides the option to enable the plugin in cmake GUIs like ccmake.
# CORE_PLUGIN enables the plugin and hides the option to disable it in
#     cmake GUIs like ccmake.

option(
  DISABLE_PLUGINS_BY_DEFAULT
  "If set to YES only plugins that are specifically enabled will be compiled"
  NO)
# This option should not show up in cmake GUIs like ccmake where all
# plugins are enabled or disabled manually.
mark_as_advanced(DISABLE_PLUGINS_BY_DEFAULT)

fast_downward_plugin(
    NAME BOOST
    HELP "Boost dependency plugin"
    DEPENDENCY_ONLY
)

fast_downward_plugin(
    NAME PLANNERNAME
    SOURCES
    countdown_timer.cc
    tiebreaking_open_list.cc
    standard_scalar_open_list.cc
    combining_evaluator.cc
    g_evaluator.cc
    sum_evaluator.cc
    weighted_evaluator.cc
    weighted_astar.cc
    enforced_hill_climbing_search.cc
    iterated_search.cc
    heuristics/blind_heuristic.cc
    heuristics/goal_count_heuristic.cc
    heuristics/max_heuristic.cc
    heuristics/additive_heuristic.cc
    heuristics/critical_path_heuristic.cc
    heuristics/ff_heuristic.cc
    heuristics/red_black_heuristic.cc
    heuristics/landmark_cut_heuristic.cc
    heuristics/landmark_heuristic.cc
    heuristics/merge_and_shrink_heuristic.cc
    heuristics/pattern_database_heuristic.cc
    pruning/dominance.cc
    pruning/strong_stubborn_sets.cc
#    DEPENDS BOOST
)

fast_downward_add_plugin_sources(PLANNER_SOURCES)

# The order in PLANNER_SOURCES influences the order in which object
# files are given to the linker, which can have a significant influence
# on performance (see issue67). The general recommendation seems to be
# to list files that define functions after files that use them.
# We approximate this by reversing the list, which will put the plugins
# first, followed by the core files, followed by the main file.
# This is certainly not optimal, but works well enough in practice.
list(REVERSE PLANNER_SOURCES)

