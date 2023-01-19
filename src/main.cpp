#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>

///< Таймаумы
#define BRAKE_BLINK 500  ///< Период мигания диода при зажатом тормозе
#define START_DELAY 500  ///< Время через которое автомобиль начинает крутить стартером
#define MIN_START_TIME 600       ///< Минимальное время запуска двигателя
#define MAX_START_TIME 3000      ///< Максимальное время время запуска двигателя
#define WITHOUT_BRAKE_TIME 3000  ///< Время на запуск без тормоза
#define STOP_TIME 2000  ///< Сколько времени после остановки двигателя работает режим АСС
#define IMMO_DELAY 150  ///< Период мигания диода, при блокировки иммобилайзера
#define IMMO_TIMES 3  ///< Количество миганий диода, при блокировки иммобилайзера
#define AFTER_START 50  ///< Какое время крутить стартером после первого сигнала запуска
#define AFTER_STOP 1000  ///< Какое время не должно быть сигнала, чтобы считать что двигатель заглох

///< Выход управления аксесуарами
#define ACC_PORT VPORTB
#define ACC_PIN PIN0_bm

///< Выход управления стартером
#define STARTER_PORT VPORTB
#define STARTER_PIN PIN1_bm

///< Выход диода на кнопке
#define BUTTON_LED_PORT VPORTB
#define BUTTON_LED_PIN PIN2_bm

///< Выход второго диода на кнопке
#define BUTTON_LED2_PORT VPORTB
#define BUTTON_LED2_PIN PIN3_bm

///< Выход управления цепью зажигания 2
// #define IGNITION2_PORT VPORTA
// #define IGNITION2_PIN PIN0_bm

///< Выход управления цепью зажигания
#define IGNITION_PORT VPORTA
#define IGNITION_PIN PIN1_bm

///< Вход паркинга
#define PARK_PORT VPORTA
#define PARK_PIN PIN2_bm

///< Вход кнопки
#define BUTTON_PORT VPORTA
#define BUTTON_PIN PIN3_bm

///< Вход состояния охраны
#define IMMO_PORT VPORTA
#define IMMO_PIN PIN4_bm

///< Вход состояния запуска
#define STARTED_PORT VPORTA
#define STARTED_PIN PIN5_bm

///< Вход педали тормоза
#define BRAKE_PORT VPORTA
#define BRAKE_PIN PIN6_bm

///< Вход дистанционного запуска
#define REMOTE_PORT VPORTA
#define REMOTE_PIN PIN7_bm

#define setOutput(port, pin) (port.DIR |= pin);
#define readPin(port, pin) (port.IN & pin)
#define writePin(port, pin, value) (((bool)value) ? (port.OUT |= pin) : (port.OUT &= ~pin))

///< Состояние авто
#define CAR_OFF 0x01       ///< Всё выключено
#define CAR_ACC 0x02       ///< Включены аксесуары
#define CAR_IGNITION 0x04  ///< Включено зажигание
#define CAR_STARTING 0x08  ///< Запуск двигателя
#define CAR_STARTED 0x10   ///< Двигатель запущен
#define CAR_STOPING 0x20   ///< Двигатель останавливается, отложеный ACC

namespace car {
uint8_t state{CAR_IGNITION};  ///< Состояние авто
bool brake{};                 ///< Нажата педаль тормоза
bool immo{};                  ///< Сигнализация в охране
bool remote{};                ///< Автомобиль заводится с сигнализации
bool remotePressed{};         ///< Сигнализация зажала кнопку
bool park{};                  ///< Авто в режиме P
bool parkUsed{};              ///< Проверять состояние режима P
#ifdef TACHO
///< Счетчики для определения оборотов двигателя
volatile uint8_t start_attempt{};
volatile uint16_t start_time{};
#endif
volatile bool started{};           ///< Есть сигнал о том, что двигатель запущен
volatile int16_t ticks{-1};        ///< Таймер для запуска/остановки
volatile int16_t stop_ticks{-1};   ///< Таймер для определения заглох ли двигатель
volatile int16_t blink_ticks{-1};  ///< Таймер для мигания светодиодом в кнопке
int16_t blink_delay{-1};           ///< Интервал мигания светодиодом
int8_t blink_count{-1};            ///< Количество миганий светодиодом в кнопке

/**
 * @brief Првоерить находится ли авто в указаных состояниях
 * @param _state Битовая маска состояний для проверки
 */
bool isState(uint8_t _state) { return state & _state; }

/**
 * @brief Обновить состояние выходов "замка зажигания"
 */
void update() {
    // Устанавливаем значения
    writePin(ACC_PORT, ACC_PIN, isState(CAR_ACC | CAR_IGNITION | CAR_STARTED | CAR_STOPING));
    writePin(IGNITION_PORT, IGNITION_PIN, isState(CAR_IGNITION | CAR_STARTING | CAR_STARTED));
    writePin(STARTER_PORT, STARTER_PIN, isState(CAR_STARTING));
}

/**
 * @brief Установить состояние авто
 * @param _state новое состояние
 */
void setState(uint8_t _state) {
    if (state == _state) return;

    if (isState(CAR_STARTING | CAR_STOPING) && ticks >= 0) ticks = -1;

    state = _state;
    update();
}

/**
 * @brief Разрешено запустить двигатель
 * @param withoutBrake не проверять состояние педали тормоза
 */
bool canStart(bool withoutBrake = false) {
    return !immo && (brake || withoutBrake) && !isState(CAR_STARTING | CAR_STARTED);
}

/**
 * @brief Запустить двигатель
 */
void turnOn() {
    setState(CAR_STARTING);
    ticks = started ? MIN_START_TIME : MAX_START_TIME;
}

/**
 * @brief Остановить двигатель
 */
void turnOff() {
    if (isState(CAR_STARTED)) {
        setState(CAR_STOPING);
        ticks = STOP_TIME;
    } else {
        setState(CAR_OFF);
    }
}

/**
 * @brief Обработка нажатия кнопки START-STOP
 */
void start() {
    if (immo && isState(CAR_OFF)) {
        blink_count = IMMO_TIMES * 2;
        blink_ticks = 0;
        blink_delay = IMMO_DELAY;
    } else if (isState(CAR_STOPING)) {
        setState(CAR_ACC);
    } else if (brake) {
        if (!isState(CAR_STARTING | CAR_STARTED)) {
            turnOn();
        } else if (!parkUsed || park) {
            turnOff();
        }
    } else {
        switch (state) {
            case CAR_OFF:
                setState(CAR_ACC);
                break;
            case CAR_ACC:
                setState(CAR_IGNITION);
                break;
            case CAR_IGNITION:
                setState(CAR_OFF);
                break;
            default:
                break;
        }
    }
}

/**
 * @brief Инициализация начального состояния авто
 * Чтение параметров и выставление "замка зажигания" в нужное положение
 */
void init() {
    brake = !readPin(BRAKE_PORT, BRAKE_PIN);
    immo = !readPin(IMMO_PORT, IMMO_PIN);
#ifndef TACHO
    started = readPin(STARTED_PORT, STARTED_PIN);
#endif
    update();
}

/**
 * @brief Цикл обработки состояния авто
 */
void loop() {
    if (!ticks) {
        ticks = -1;
        if (isState(CAR_STARTING)) {
            setState(started ? CAR_STARTED : CAR_IGNITION);
        } else if (isState(CAR_STOPING)) {
            setState(CAR_OFF);
        }
    }

    if (!stop_ticks) {
        stop_ticks = -1;
#ifdef TACHO
        started = false;
        start_time = start_attempt = 0;
#endif
        setState(CAR_IGNITION);
    }

    park = readPin(PARK_PORT, PARK_PIN);
    ///< Запоминаем что был сигнал постановки на паркинг
    if (park && !parkUsed) parkUsed = true;
    immo = !readPin(IMMO_PORT, IMMO_PIN);
    if (brake != !readPin(BRAKE_PORT, BRAKE_PIN)) {
        brake = !brake;
        if (!(immo && isState(CAR_OFF))) {
            if (brake) {
                blink_count = -1;
                blink_ticks = 0;
                blink_delay = BRAKE_BLINK;
            } else {
                blink_ticks = -1;
            }
        }
    }
#ifdef TACHO
    if (started && isState(CAR_STARTING | CAR_IGNITION)) {
        setState(CAR_STARTED);
    } else if (!started && isState(CAR_STARTED)) {
        setState(CAR_IGNITION);
    }
#else
    if (started != readPin(STARTED_PORT, STARTED_PIN)) {
        if (!started) {
            if (isState(CAR_STARTING | CAR_IGNITION)) setState(CAR_STARTED);
            stop_ticks = -1;
            started = true;
        } else {
            stop_ticks = AFTER_STOP;
            started = false;
        }
    }
#endif

    ///< Запуск с сигнализации
    if (!readPin(REMOTE_PORT, REMOTE_PIN)) {
        if (!remotePressed) {
            remotePressed = true;
            if (isState(CAR_STARTING | CAR_STARTED) || remote) {
                setState(CAR_OFF);
                remote = false;
            } else if (!isState(CAR_STARTING | CAR_STARTED)) {
                remote = true;
                turnOn();
            }
        }
    } else if (remotePressed) {
        remotePressed = false;
    }

    writePin(BUTTON_LED2_PORT, BUTTON_LED2_PIN, (!immo || remotePressed));
    if (!blink_ticks) {
        if (blink_count > 0) --blink_count;
        blink_ticks = blink_count ? blink_delay : -1;
        writePin(BUTTON_LED_PORT, BUTTON_LED_PIN, (!readPin(BUTTON_LED_PORT, BUTTON_LED_PIN)));
    } else if (blink_ticks < 0) {
        writePin(BUTTON_LED_PORT, BUTTON_LED_PIN, isState(CAR_STARTED));
    }
}

}  // namespace car

namespace button {
bool pressed = false;        ///< Нажата ли кнопка
bool sticky = false;         ///< Сработало зажатие кнопки
volatile int16_t ticks{-1};  ///< Таймер зажатия кнопки

/**
 * @brief Цикл обработки кнопки
 */
void loop() {
    if (!ticks) {
        ticks = -1;
        sticky = true;
        if (car::canStart(true))
            car::turnOn();
        else
            car::turnOff();
    }
    if (pressed == !readPin(BUTTON_PORT, BUTTON_PIN)) return;
    pressed = !pressed;
    if (pressed) {
        ticks = car::canStart() ? START_DELAY : WITHOUT_BRAKE_TIME;
        return;
    }
    if (sticky) {
        sticky = false;
    } else {
        ticks = -1;
        car::start();
    }
}

}  // namespace button

/**
 * @brief Инициализация контроллера
 */
void init() {
    // Изменяем множитель на переферию 1/64
    uint8_t tmp = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm;
    CPU_CCP |= CCP_IOREG_gc;
    CLKCTRL_MCLKCTRLB = tmp;

    ///<
    setOutput(ACC_PORT, ACC_PIN);
    setOutput(IGNITION_PORT, IGNITION_PIN);
    setOutput(STARTER_PORT, STARTER_PIN);
    setOutput(BUTTON_LED_PORT, BUTTON_LED_PIN);
    setOutput(BUTTON_LED2_PORT, BUTTON_LED2_PIN);

    cli();
    ///< Настройки таймера счетчика А
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;  ///< Включить таймер
    TCA0.SINGLE.PER = 625;                      ///< Период таймера. (20MHz / 64) / 625 = 1ms
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;   ///< Включить прерывание
#ifdef TACHO
    ///< Настройки событий
    EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_PORTA_PIN5_gc;    ///< Генератор для ASYNCCH0 - PA5
    EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc;  ///< ASYNCUSER0 - это Таймер B

    ///< Настройки таймера счетчика B
    TCB0.CTRLA |= TCB_ENABLE_bm;                 ///< Включить таймер
    TCB0.CTRLB |= TCB_CNTMODE_FRQ_gc;            ///< Режим - захват частоты
    TCB0.EVCTRL |= TCB_EDGE_bm | TCB_CAPTEI_bm;  ///< Включить события по нисходящему фронту
    TCB0.INTCTRL |= TCB_CAPT_bm;                 ///< Включить прерывание по захвату
#endif
    sei();

    car::init();
}

int main() {
    init();
    for (;;) {
        button::loop();
        car::loop();
    }
    return 0;
}

#ifdef TACHO
///< Прерывание таймера/счетчика B
ISR(TCB0_INT_vect) {
    TCB0.INTFLAGS |= TCB_CAPT_bm;
    car::start_time += TCB0.CCMP / 625;
    car::start_attempt++;
    if (car::start_attempt == 3) {
        car::started = car::start_time < 120;
        car::stop_ticks = AFTER_STOP;
        car::start_attempt = car::start_time = 0;
    }
}
#endif

///< Прурывание таймера/счетчика А
ISR(TCA0_OVF_vect) {
    TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_OVF_bm;
    if (button::ticks > 0) button::ticks--;
    if (car::ticks > 0) car::ticks--;
    if (car::stop_ticks > 0) car::stop_ticks--;
    if (car::blink_ticks > 0) car::blink_ticks--;
}