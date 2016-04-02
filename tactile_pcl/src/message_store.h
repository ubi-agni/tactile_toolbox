#pragma once
#include <deque>
#include <map>
#include <string>

template <typename T, typename Store = std::deque<T> >
class MessageStore
{
public:
	typedef typedef std::map<std::string, Store> MapType;
	MessageStore();
	void clear() { msgs_.clear(); }
	void push_back(const std::string &id, const T &value) {
		MapType::iterator it = msgs_.find(id);
		if (it == msgs_.end()) it = msgs_.insert(std::make_pair(id, Store())).second;
		it->push_back(value);
	}

protected:
	MapType msgs_;
};
