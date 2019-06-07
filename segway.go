package main

import (
    "fmt"
	"time"
    "math"
    "os"
	"os/exec"
    "regexp"
	"runtime"
    "strconv"
    "strings"

    "gobot.io/x/gobot"
    "gobot.io/x/gobot/drivers/i2c"
	"gobot.io/x/gobot/platforms/chip"
	
    "github.com/stianeikeland/go-rpio"
    
    "golang.org/x/sys/unix"
)

const ACCEL_SENS = 16.384
const GYRO_SENS = 131

var reg = regexp.MustCompile(`\[p='([0-9]+?\.[0-9]{3,}?)';i='([0-9]+?\.[0-9]{3,}?)';d='([0-9]+?\.[0-9]{3,}?)'\]`)
var (
    Accelerometer  i2c.ThreeDDataFloat
    Gyroscope      i2c.ThreeDDataFloat
    previousTime   time.Time
    currentTime    time.Time
    elapsedTime    time.Duration
    GyroXSlice     []float64
    elapsedSeconds float64
    lcX            []float64 = make([]float64, 40)
    lcY            []float64 = make([]float64, 40)
    lcXM           float64
    lcYM           float64
    gyroAngleX     float64
    gyroAngleY     float64
    yaw            float64
    pitch          float64
    roll           float64
)

type PID struct {
    P float64
    I float64
    D float64
}

var (
    kP                 float64 = 15
    kI                 float64 = 5
    kD                 float64 = 3
    pid                PID
    PIDt               float64
    errorAngle         float64
    previousErrorAngle float64
    desiredAngle       float64 = 0
)

const (
    DStatus = "[STATUS] "
    DData   = "[ DATA ] "
    DRead   = "[ READ ]"
    DMsg    = "[ MSG  ] "
    DInput  = "[ INPUT] "
    DTest   = "[ TEST ]"
    DInfo   = "[ INFO ] "
)

var clear map[string]func()

func init() {
    clear = make(map[string]func())
    clear["linux"] = func() { 
        cmd := exec.Command("clear")
        cmd.Stdout = os.Stdout
        cmd.Run()
    }
    clear["windows"] = func() {
        cmd := exec.Command("cmd", "/c", "cls")
        cmd.Stdout = os.Stdout
        cmd.Run()
    }
}

func CallClear() {
    value, ok := clear[runtime.GOOS]
    if ok {
        value()
    } else {
        panic("Your platform is unsupported! I can't clear terminal screen :(")
    }
}


func main() {
	if err := rpio.Open(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}


	pinStepA := rpio.Pin(5)
	pinDirA := rpio.Pin(6)
	pinStepB := rpio.Pin(19)
	pinDirB := rpio.Pin(26)

	pinEnable := rpio.Pin(13)
	
	pinEnable.Output()
	pinStepA.Output()
	pinDirA.Output()
	pinStepB.Output()
	pinDirB.Output()


	defer rpio.Close()

    board := chip.NewAdaptor()
    mpu6050 := i2c.NewMPU6050Driver(board, 1000)

    work := func() {
        gobot.Every(10*time.Millisecond, func() {
            start:
            mpu6050.GetData()

            Accelerometer = i2c.ThreeDDataFloat {
                X: (float64(mpu6050.Accelerometer.X) / 16384.0),
                Y: (float64(mpu6050.Accelerometer.Y) / 16384.0),
                Z: (float64(mpu6050.Accelerometer.Z) / 16384.0),
            }
            
            Accelerometer.X = (math.Atan((Accelerometer.Y) / math.Sqrt(math.Pow((Accelerometer.X), 2) + math.Pow((Accelerometer.Z), 2))) * 180 / math.Pi) + (mpu6050.AccError.X * -1)
            Accelerometer.Y = (math.Atan((Accelerometer.X) / math.Sqrt(math.Pow((Accelerometer.Y), 2) + math.Pow((Accelerometer.Z), 2))) * 180 / math.Pi) + (mpu6050.AccError.Y * -1)
            
            previousTime = time.Now()
            currentTime = time.Now()
            elapsedTime = currentTime.Sub(previousTime)
            elapsedSeconds = float64(elapsedTime.Nanoseconds()) / 1000000000
            
            Gyroscope = i2c.ThreeDDataFloat {
                X: (float64(mpu6050.Gyroscope.X) / 131.0),
                Y: (float64(mpu6050.Gyroscope.Y) / 131.0),
                Z: (float64(mpu6050.Gyroscope.Z) / 131.0),
            }
            
            // Correct the outputs with the calculated error values
            Gyroscope.X = Gyroscope.X + (mpu6050.GyroError.X * -1)
            Gyroscope.Y = Gyroscope.Y + (mpu6050.GyroError.Y * -1)
            Gyroscope.Z = Gyroscope.Z + (mpu6050.GyroError.Z * -1)
            
            // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
            gyroAngleX = gyroAngleX + Gyroscope.X * (0.01 + elapsedSeconds) // deg/s * s = deg
            gyroAngleY = gyroAngleY + Gyroscope.Y * (0.01 + elapsedSeconds)
            yaw =  yaw + Gyroscope.Z * (0.01 + elapsedSeconds)
            // Complementary filter - combine acceleromter and gyro angle values
            roll = 0.98 * gyroAngleX + 0.02 * Accelerometer.X
            pitch = 0.98 * gyroAngleY + 0.02 * Accelerometer.Y
            
            lcX = append(lcX[1:], Gyroscope.X)
            lcY = append(lcY[1:], Gyroscope.Y)
            
            for i := 0; i<40; i++ {
                lcXM += lcX[i]
                lcYM += lcY[i]
            }
            
            lcXM = lcXM / 40
            lcYM = lcYM / 40
            
            if lcXM < 0.12 && ((roll > 1 && roll < 3) || (roll < -1 && roll > -3)) {
                gyroAngleX += gyroAngleX *-0.8
            }
            
            if lcYM < 0.12 && ((pitch > 1 && pitch < 3) || (pitch < -1 && pitch > -3)) {
                gyroAngleY += gyroAngleY *-0.8
            }
            
            errorAngle = desiredAngle - pitch
            
            pid.P = errorAngle
            
            if -5 < errorAngle && errorAngle < 5 {
                pid.I = pid.I + (errorAngle * (0.01 + elapsedSeconds))
            }
            
            pid.D = ((errorAngle - previousErrorAngle) / (0.01 + elapsedSeconds))
            
            PIDt = kP*pid.P + kI*pid.I + kD*pid.D
            
            CallClear()
            
            previousErrorAngle = errorAngle
            
            fmt.Printf("Correction:\n\tX: %+5.3f\n\tY: %+5.3f\n\nGyro:\n\tX: %+05.03f | %+05.03f | %+04.03f\n\tY: %+04.03f | %+04.03f\n\nAccelerometer:\n\tX: %+3.3f\n\tY: %+3.3f\n\nYaw:   %+3.3f\nRoll:  %+3.3f\nPitch: %+3.3f\n", lcXM, lcYM, gyroAngleX, (float64(mpu6050.Gyroscope.X) / 131.0)* (0.1 + elapsedSeconds), Gyroscope.X, gyroAngleY, (float64(mpu6050.Gyroscope.Y) / 131.0)* (0.1 + elapsedSeconds), Accelerometer.X, Accelerometer.Y, yaw, roll, pitch)
		})
		gobot.Every(250*time.Microsecond, func() {
			t, err := time.ParseDuration((strconv.Itoa(int(500 - math.Abs(PIDt))) + "us" ))
			if err != nil {
				t = 500*time.Microsecond
			}

			if math.Abs(errorAngle) > 3 {
				pinEnable.Low()

				if errorAngle < 0 {
                    pinDirA.Low()
                    pinDirB.High()
				} else {
                    pinDirA.High()
                    pinDirB.Low()
				}
				
                pinStepA.High()
                pinStepB.High()
				<-time.After(t)
                pinStepA.Low()
                pinStepB.High()
				<-time.After(t)
			} else {
				pinEnable.High()
			}
        })
        fd, _ := unix.Socket(unix.AF_BLUETOOTH, unix.SOCK_STREAM, unix.BTPROTO_RFCOMM)
        _ = unix.Bind(fd, &unix.SockaddrRFCOMM{
            Channel: 1,
            Addr:    [6]uint8{0, 0, 0, 0, 0, 0}, // BDADDR_ANY or 00:00:00:00:00:00
        })
        gobot.Every(time.Second, func() {
            start:
            for {
                _ = unix.Listen(fd, 1)
                nfd, _, _ := unix.Accept(fd)
                unix.Write(nfd, []byte("\n" + string([]byte{0x0D}) + DStatus +  "Connected"))
                data := make([]byte, 1024)
                var msg, sendmsg, dt string
                var tmsg string = "\n" + string([]byte{0x0D}) + DTest + " "
                for true {
                    data = make([]byte, 1024)
                    if _, err := unix.Write(nfd, []byte(tmsg)); err != nil {
                        // Assume connection was closed.
                        goto start
                    }
                    sendmsg = strings.TrimSpace(sendmsg)
                    msg = ""
                    for true {
                        if _, err := unix.Read(nfd, data); err != nil {
                            // Assume connection was closed.
                            goto start
                        }
                        if (data[0] == 0x0D && data[1] == 0x0A) {
                            break
                        }
                        msg += string(data[0])
                    }
                    if strings.ToLower(msg) == "restart" {

                    } else if strings.ToLower(msg) == "disconnect" {
                        unix.Write(nfd, []byte("\n" + string([]byte{0x0D}) + DStatus + "Disconnecting..." + "\n" + string([]byte{0x0D})))
                        unix.Close(nfd)
                        goto start
                    } else if strings.HasPrefix(msg, "sd#") {
                        dt = strings.TrimSpace(strings.Replace(msg, "sd#", "", -1))
                        if !reg.MatchString(dt) {
                            // Bericht dat format niet klopt...
                            sendmsg = "Invalid format...\n" + string([]byte{0x0D}) + "Please write the data in the following format:\n" + string([]byte{0x0D}) + "[P='1.000';I='2.000';D='3.000']"
                            unix.Write(nfd, []byte(sendmsg))
                        } else {
                            sP := reg.ReplaceAllString(dt, "$1")
                            sI := reg.ReplaceAllString(dt, "$2")
                            sD := reg.ReplaceAllString(dt, "$3")


                            if nP, err := strconv.ParseFloat(sP, 64); err != nil {
                                sendmsg = "Invalid P value...\n" + string([]byte{0x0D}) + "Please write the data in the following format:\n" + string([]byte{0x0D}) + "[P='1.000';I='2.000';D='3.000']"
                                unix.Write(nfd, []byte(sendmsg))
                            } else if nI, err := strconv.ParseFloat(sI, 64); err != nil {
                                sendmsg = "Invalid I value...\n" + string([]byte{0x0D}) + "Please write the data in the following format:\n" + string([]byte{0x0D}) + "[P='1.000';I='2.000';D='3.000']"
                                unix.Write(nfd, []byte(sendmsg))
                            } else if nD, err := strconv.ParseFloat(sD, 64); err != nil {
                                sendmsg = "Invalid D value...\n" + string([]byte{0x0D}) + "Please write the data in the following format:\n" + string([]byte{0x0D}) + "[P='1.000';I='2.000';D='3.000']"
                                unix.Write(nfd, []byte(sendmsg))
                            } else {
                                kP = nP
                                kI = nI
                                kD = nD

                                sendmsg = "PID Succesfully updated!\n"
                                unix.Write(nfd, []byte(sendmsg))
                            }
                        }
                    } else if strings.ToLower(msg) == "getdata" {
                        sendmsg = "\n" + string([]byte{0x0D}) + DRead + "\n" + string([]byte{0x0D}) + "[" + "\n" + string([]byte{0x0D}) + "\tP: " + strconv.FormatFloat(kP, 'f', 3, 64) + ",\n" + string([]byte{0x0D}) + "\tI: " + strconv.FormatFloat(kI, 'f', 3, 64) + ",\n" + string([]byte{0x0D}) + "\tD: " + strconv.FormatFloat(kD, 'f', 3, 64) + "\n" + string([]byte{0x0D}) + "]"
                        unix.Write(nfd, []byte(sendmsg))
                    }
                }
            }
        })
    }

    robot := gobot.NewRobot("mpu6050Bot",
        []gobot.Connection{board},
        []gobot.Device{mpu6050},
        work,
    )
	
	pinEnable.High()

	<-time.After(500*time.Millisecond)

	pinEnable.Low()

    pinDirA.Low()
    pinDirB.Low()
	

	robot.Start()


	fmt.Println(mpu6050.GyroError)
	fmt.Println(mpu6050.AccError)
	pinEnable.High()
}