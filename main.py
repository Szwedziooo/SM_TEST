import tkinter as tk
import serial

# Utworzenie połączenia z portem szeregowym
ser = serial.Serial('COM4', 115200)

# Globalne zmienne do przechowywania ostatnich ustawień PID
last_kp = 0.0
last_ki = 0.0
last_kd = 0.0

def send_rpm():
    global last_kp, last_ki, last_kd
    try:
        rpm_value = int(rpm_entry.get())
        if 0 <= rpm_value <= 15000:
            command = f"SET:{rpm_value}:{last_kp}:{last_ki}:{last_kd}\n"
            ser.write(command.encode())
            status_label.config(text=f"Wysłano: RPM={rpm_value}")
        else:
            status_label.config(text="Wartość RPM poza zakresem 0-15000!")
    except ValueError:
        status_label.config(text="Proszę wprowadzić poprawną wartość liczbową dla RPM")

def send_all_parameters():
    global last_kp, last_ki, last_kd
    try:
        rpm_value = int(rpm_entry.get())
        last_kp = float(kp_entry.get())
        last_ki = float(ki_entry.get())
        last_kd = float(kd_entry.get())

        if 0 <= rpm_value <= 15000:
            command = f"SET:{rpm_value}:{last_kp}:{last_ki}:{last_kd}\n"
            ser.write(command.encode())
            status_label.config(text=f"Wysłano: RPM={rpm_value}, Kp={last_kp}, Ki={last_ki}, Kd={last_kd}")
        else:
            status_label.config(text="Wartość RPM poza zakresem 0-15000!")
    except ValueError:
        status_label.config(text="Proszę wprowadzić poprawne wartości liczbowe")

root = tk.Tk()
root.title("RPM i Regulator PID")

# Ramka dla RPM
frame_rpm = tk.Frame(root)
frame_rpm.pack(padx=10, pady=10)

tk.Label(frame_rpm, text="RPM:").pack(side=tk.LEFT)
rpm_entry = tk.Entry(frame_rpm)
rpm_entry.pack(side=tk.LEFT)
tk.Button(frame_rpm, text="Potwierdź RPM", command=send_rpm).pack(side=tk.LEFT)

# Ramka dla PID
frame_pid = tk.Frame(root)
frame_pid.pack(padx=10, pady=10)

tk.Label(frame_pid, text="Kp:").pack(side=tk.LEFT)
kp_entry = tk.Entry(frame_pid)
kp_entry.pack(side=tk.LEFT)

tk.Label(frame_pid, text="Ki:").pack(side=tk.LEFT)
ki_entry = tk.Entry(frame_pid)
ki_entry.pack(side=tk.LEFT)

tk.Label(frame_pid, text="Kd:").pack(side=tk.LEFT)
kd_entry = tk.Entry(frame_pid)
kd_entry.pack(side=tk.LEFT)

# Przycisk do wysyłania wszystkich parametrów
tk.Button(root, text="Wyślij Wszystkie Parametry", command=send_all_parameters).pack()

# Etykieta wyświetlająca status
status_label = tk.Label(root, text="Status: Nie wysłano danych")
status_label.pack(pady=10)

root.mainloop()
