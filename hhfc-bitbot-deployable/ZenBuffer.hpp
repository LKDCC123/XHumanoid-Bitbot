#include <vector>
#include <iostream>

namespace zzs
{
    template <typename T>
    class RingBuffer
    {
    public:
        RingBuffer(int _Size)
        {
            this->Size = _Size;
            this->Data.resize(_Size);
            this->CurrentIndex = 0;
        }

        RingBuffer(int _Size, T& object)
        {
            this->Size = _Size;
            this->Data.reserve(this->Size);
            for (size_t i = 0; i < this->Size; i++)
            {
                this->Data.push_back(object); // need to push object multiple times, so can not use move here.
            }
            this->CurrentIndex = 0;
        }

        void push(const T& _Data)
        {
            this->Data[this->CurrentIndex] = _Data;
            this->CurrentIndex = (this->CurrentIndex + 1) % this->Size;
        }

        T& operator()(int _Index)
        {
            return this->Data[(_Index + this->CurrentIndex) % this->Size];
        }

        T& operator[](int _Index)
        {
            return this->Data[(_Index + this->CurrentIndex) % this->Size];
        }
        // get oldest data
        T& front()
        {
            return this->Data[this->CurrentIndex];
        }
        // get newest data
        T& back()
        {
            return this->Data[(this->CurrentIndex - 1 + this->Size) % this->Size];
        }

        int size()
        {
            return this->Size;
        }

    private:
        std::vector<T> Data;
        int Size;
        int CurrentIndex;
    };


    template<typename T>
    class filter
    {
    public:
        filter(int buffer_sz = 3)
        {
            this->buffer_sz = buffer_sz;
            this->buffer.resize(buffer_sz);
            for (auto& i : this->buffer)
                i = 0;
        }
        void clear()
        {
            for (auto& i : this->buffer)
            {
                i = 0;
            }
        }

        float operator()(T input)
        {
            buffer[this->pointer] = input;
            this->pointer++;
            this->pointer %= this->buffer_sz;
            float sum = 0;
            for (auto& i : this->buffer)
            {
                sum += i;
            }
            return sum / buffer_sz;
        }

    private:
        int buffer_sz;
        std::vector<T> buffer;
        int pointer = 0;
    };

    using filterd = filter<double>;
    using filterf = filter<float>;
};