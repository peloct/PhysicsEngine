#pragma once

template <typename... Args>
class IEventHandler
{
public:
	virtual void Invoke(Args... args) = 0;
};

template <typename S, typename... Args>
class EventHandler : public IEventHandler<Args...>
{
public:
	typedef void Function(S, Args...);

	EventHandler(S self, Function* function) : IEventHandler<Args...>(), self(self), function(function) {}

	void Invoke(Args... args) override
	{
		function(self, args...);
	}

private:
	S self;
	Function* function;
};