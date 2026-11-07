import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise SystemExit("Joystick belum dicolok!")

joy = pygame.joystick.Joystick(0)
joy.init()

screen = pygame.display.set_mode((680, 480))
pygame.display.set_caption(f"Joystick Tester - {joy.get_name()}")
font = pygame.font.SysFont("Consolas", 18)
big = pygame.font.SysFont("Consolas", 22, bold=True)
clock = pygame.time.Clock()

active_axis_info = "Dorong salah satu stick sampai mentok..."
last_buttons = []

running = True
while running:
    pygame.event.pump()
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
            running = False

    screen.fill((18, 22, 32))
    screen.blit(big.render(f"Controller: {joy.get_name()[:35]}", True, (80, 215, 255)), (20, 15))

    # --- 1. TAMPILAN BAR VISUAL AXIS ---
    num_axes = joy.get_numaxes()
    for i in range(num_axes):
        val = joy.get_axis(i)
        y = 65 + i * 42

        # Deteksi otomatis kalau stick didorong cukup jauh (> 50%)
        if abs(val) > 0.5:
            arah = "MINUS (-)" if val < 0 else "PLUS (+)"
            inv_saran = "True" if val < 0 else "False"
            active_axis_info = f"Terdeteksi: AXIS {i} ({arah}) -> Jika ini MAJU, set AXIS_FWD={i}, INV_FWD={inv_saran}"

        # Warna teks berubah kuning terang kalau axis sedang bergerak
        is_moving = abs(val) > 0.15
        txt_col = (255, 220, 0) if is_moving else (170, 180, 200)
        screen.blit(font.render(f"AXIS {i} : {val:+.1f}", True, txt_col), (20, y + 4))

        # Gambar kotak bar (Tengah = 0.0, Kiri = -1.0, Kanan = +1.0)
        pygame.draw.rect(screen, (35, 45, 65), (170, y, 400, 26), border_radius=4)
        pygame.draw.line(screen, (150, 160, 180), (370, y - 2), (370, y + 28), 2)

        bar_w = int(val * 200)
        bar_col = (70, 220, 130) if val >= 0 else (255, 140, 60)
        if abs(val) > 0.05:
            pygame.draw.rect(screen, bar_col, (370 + min(0, bar_w), y, abs(bar_w), 26), border_radius=4)

    # --- 2. KOTak KESIMPULAN OTOMATIS ---
    y_box = 75 + num_axes * 42
    pygame.draw.rect(screen, (28, 38, 55), (20, y_box, 640, 45), border_radius=6)
    screen.blit(font.render(active_axis_info, True, (100, 255, 160)), (30, y_box + 13))

    # --- 3. TAMPILAN TOMBOL YANG DITEKAN ---
    pressed = [str(b) for b in range(joy.get_numbuttons()) if joy.get_button(b)]
    if pressed:
        last_buttons = pressed
    btn_str = ", ".join(pressed) if pressed else (f"Terakhir ditekan: {', '.join(last_buttons)}" if last_buttons else "Belum ada")
    screen.blit(big.render(f"Tombol Ditekan : {btn_str}", True, (255, 200, 90)), (20, y_box + 65))

    pygame.display.flip()
    clock.tick(15)

pygame.quit()