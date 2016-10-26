# -*- coding: utf-8 -*-
from __future__ import print_function

import os

ALIASES = {}

# example for an alias called goal-count that runs weighted A* with a goal 
# counting heuristic:
ALIASES["goal-count"] = ["--search", "wastar(gc())"]

# you can test this alias by running 
# ./fast-downward.py --alias competition <problem-file>
# please enter empty strings if you do not want to submit 4 planners
ALIASES["optimal_1"] = ["TODO: insert your configuration here"]
ALIASES["optimal_2"] = ["TODO: insert your configuration here"]

ALIASES["satisficing_1"] = ["TODO: insert your configuration here"]
ALIASES["satisficing_2"] = ["TODO: insert your configuration here"]


def show_aliases():
    for alias in sorted(ALIASES.keys()):
        print(alias)


def set_options_for_alias(alias_name, args):
    """
    If alias_name is an alias for a configuration, set args.search_options
    to the corresponding command-line arguments. Otherwise raise KeyError.
    """
    assert not args.search_options

    if alias_name in ALIASES:
        args.search_options = ALIASES[alias_name]
    else:
        raise KeyError(alias_name)
