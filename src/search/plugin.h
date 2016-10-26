#ifndef PLUGIN_H
#define PLUGIN_H

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include "standard_scalar_open_list.h"
#include "tiebreaking_open_list.h"

template <class T>
class Plugin {
    Plugin(const Plugin<T> &copy);
public:
    Plugin(const std::string &key, typename Registry<T *>::Factory factory) {
        Registry<T *>::
        instance()->register_object(key, factory);
    }
    ~Plugin() {}
};

template <class Entry>
class Plugin<OpenList<Entry > > {
    Plugin(const Plugin<OpenList<Entry > > &copy);
public:
    ~Plugin();

    static void register_open_lists() {
        Registry<OpenList<Entry > *>::instance()->register_object(
            "single", StandardScalarOpenList<Entry>::_parse);
        Registry<OpenList<Entry > *>::instance()->register_object(
            "tiebreaking", TieBreakingOpenList<Entry>::_parse);
    }
};

#endif
