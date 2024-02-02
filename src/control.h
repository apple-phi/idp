
namespace Control
{
    class FirstOrderFilter
    {
    public:
        explicit FirstOrderFilter(const float alpha = 0.1);
        float filter(float input);
        void reset();

    private:
        float alpha;
        float output, input;
    };

    class PID
    {
    public:
        PID(float kc, float Ti, float Td, FirstOrderFilter derivativeFilter = FirstOrderFilter(0.1));
        float update(float error, float dt);
        void reset();

    private:
        FirstOrderFilter derivativeFilter;
        float kc, Ti, Td;
        float integral;
        float prevError;
    };
}