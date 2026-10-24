"""Aplikasi pygame: loop utama yang menyambungkan masukan, Engine, dan gambar.

Jalankan dari folder proyek:  python -m sim.app
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import pygame

from ..config import Config, ConfigError, Scenario, load_config, load_scenario
from ..engine import Engine
from ..types import Frame, Intent, Velocity
from .input import InputMapper, UiCommands
from .layers import LayerRegistry, RenderContext, draw_background, draw_foreground
from .panel import PANEL_WIDTH, Panel
from .viewport import Viewport

ROOT = Path(__file__).resolve().parents[2]
TARGET_FPS = 60
MAX_DT = 0.05  # dt dipotong supaya jeda panjang tidak melempar objek jauh

NOTICE_COLOR = (255, 235, 120)
ERROR_COLOR = (255, 120, 120)


class App:
    def __init__(self, config_path: Path, scenario_path: Path, window: tuple[int, int] = (1280, 720)):
        self.config_path = Path(config_path)
        self.scenario_path = Path(scenario_path)
        self.cfg: Config = load_config(self.config_path)  # galat awal dibiarkan naik
        scenario: Scenario = load_scenario(self.scenario_path, self.cfg)

        pygame.init()
        pygame.display.set_caption("Simulasi Obstacle Avoidance: DAVG + cf-MPC (KKA kelompok 8)")
        self.screen = pygame.display.set_mode(window, pygame.RESIZABLE)
        names = "dejavusans,liberationsans,arial"
        self.font = pygame.font.SysFont(names, 15)
        self.title_font = pygame.font.SysFont(names, 17, bold=True)

        self.engine = Engine(self.cfg, scenario)
        self.view = Viewport(self.cfg.field_width, self.cfg.field_height)
        self.registry = LayerRegistry(scenario.layers)
        self.panel = Panel(self.registry, self.font, self.title_font)
        self.mapper = InputMapper(self.view)
        self.frame: Frame = self.engine.tick(0.0)
        self._banner: Optional[tuple[str, tuple[int, int, int], float]] = None
        self._set_banner(f"Skenario: {scenario.name}", NOTICE_COLOR, 4.0)

    # --------------------------------------------------------------------- muat ulang ---

    def reload(self) -> None:
        """R: baca ulang config dan skenario; galat ditampilkan, keadaan lama dipertahankan."""
        try:
            cfg = load_config(self.config_path)
            scenario = load_scenario(self.scenario_path, cfg)
        except ConfigError as e:
            self._set_banner(str(e), ERROR_COLOR, 8.0)
            return
        self.cfg = cfg
        self.engine = Engine(cfg, scenario)
        self.view = Viewport(cfg.field_width, cfg.field_height)
        self._fit_view(self.screen.get_size())
        self.mapper = InputMapper(self.view)
        self.registry.set_states(scenario.layers)
        self.panel.blur_all()
        self.frame = self.engine.tick(0.0)
        self._set_banner(f"Dimuat ulang: {scenario.name}", NOTICE_COLOR, 3.0)

    def _set_banner(self, text: str, color: tuple[int, int, int], seconds: float) -> None:
        self._banner = (text, color, pygame.time.get_ticks() / 1000.0 + seconds)

    # ------------------------------------------------------------------------ loop ---

    def _fit_view(self, window_size: tuple[int, int]) -> None:
        width = window_size[0] - (PANEL_WIDTH if self.panel.visible else 0)
        self.view.fit(0, 0, max(1, width), window_size[1])

    def run(self) -> None:
        self.loop()
        pygame.quit()

    def loop(self, max_frames: Optional[int] = None) -> None:
        """Jalan sampai jendela ditutup, atau sebanyak max_frames frame (untuk uji asap)."""
        clock = pygame.time.Clock()
        frames = 0
        while True:
            dt = min(clock.tick(TARGET_FPS) / 1000.0, MAX_DT)
            window_size = self.screen.get_size()
            self._fit_view(window_size)

            intent, ui = Intent(), UiCommands()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    ui.quit = True
                    continue
                consumed, committed = self.panel.handle_event(event, self.frame, window_size)
                intent.set_velocities.extend(committed)
                if not consumed:
                    self.mapper.handle(event, self.frame, intent, ui)
            if ui.quit:
                break
            if ui.toggle_panel:
                self.panel.toggle()
            for key in ui.layer_keys:
                self.registry.toggle_by_key(key)
            if ui.reload:
                self.reload()
                intent = Intent()
            else:
                intent.manual = (Velocity() if self.panel.editing
                                 else self.mapper.manual_command(pygame.key.get_pressed()))

            self.frame = self.engine.tick(dt, intent)
            if self.frame.notice:
                self._set_banner(self.frame.notice, NOTICE_COLOR, 3.0)

            self._draw(clock.get_fps(), window_size)
            pygame.display.flip()
            frames += 1
            if max_frames is not None and frames >= max_frames:
                break

    def _draw(self, fps: float, window_size: tuple[int, int]) -> None:
        ctx = RenderContext(self.screen, self.view, self.cfg, fps, self.font)
        draw_background(ctx)
        self.registry.draw_all(self.frame, ctx)
        draw_foreground(self.frame, ctx)
        self.panel.draw(self.screen, self.frame)
        self._draw_banner(window_size)

    def _draw_banner(self, window_size: tuple[int, int]) -> None:
        if self._banner is None:
            return
        text, color, expires = self._banner
        if pygame.time.get_ticks() / 1000.0 > expires:
            self._banner = None
            return
        x, _, w, _ = self.view.field_rect()
        label = self.font.render(text, True, color)
        max_width = max(1, w - 20)
        if label.get_width() > max_width:
            label = label.subsurface((0, 0, max_width, label.get_height()))
        self.screen.blit(label, (x + 10, window_size[1] - 28))


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Simulasi obstacle avoidance humanoid (DAVG + cf-MPC)")
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--scenario", type=Path, default=ROOT / "scenarios" / "default.json")
    args = parser.parse_args(argv)
    try:
        App(args.config, args.scenario).run()
    except ConfigError as e:
        print(f"Galat konfigurasi: {e}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
