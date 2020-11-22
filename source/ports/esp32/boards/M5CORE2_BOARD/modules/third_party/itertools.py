# MIT License
# 
# Copyright (c) 2019 Johan Brichau
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
def count(start=0, step=1):
    n = start
    while True:
        yield n
        n += step

def cycle(iterable):
    try:
        len(iterable)
    except TypeError:
        # len() is not defined for this type. Assume it is
        # a finite iterable so we must cache the elements.
        saved = []
        for element in iterable:
            yield element
            saved.append(element)
    while saved:
        yield from saved

def repeat(obj, times=None):
    if times is None:
        while True:
            yield obj
    else:
        for i in range(times):
            yield obj

def chain(*iterables):
    for i in iterables:
        yield from i

def islice(p, start, stop=(), step=1):
    if stop == ():
        stop = start
        start = 0
    # TODO: optimizing or breaking semantics?
    if start >= stop:
        return
    it = iter(p)
    for i in range(start):
        next(it)

    while True:
        yield next(it)
        for i in range(step - 1):
            next(it)
        start += step
        if start >= stop:
            return

def tee(iterable, n=2):
    return [iter(iterable)] * n

def starmap(function, iterable):
    for args in iterable:
        yield function(*args)

def accumulate(iterable, func=lambda x, y: x + y):
    it = iter(iterable)
    try:
        acc = next(it)
    except StopIteration:
        return
    yield acc
    for element in it:
        acc = func(acc, element)
        yield acc
