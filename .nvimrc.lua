-- .nvimrc.lua — Arduino workspace config for asward/workspace-config.nvim
-- Requires: arduino-cli on PATH, board core installed (e.g. `arduino-cli core install arduino:avr`),
-- and clangd on PATH (`:MasonInstall clangd`) — arduino-language-server spawns it internally.

-- ── Project settings: edit these ──────────────────────────────────────────
local FQBN = "arduino:avr:uno" -- `arduino-cli board listall` to find yours
local PORT = "/dev/ttyACM0" -- `arduino-cli board list` to find yours
local BAUD = "9600"
-- ──────────────────────────────────────────────────────────────────────────

local is_mac = vim.fn.has("mac") == 1
local cli_config = vim.fn.expand(is_mac and "~/Library/Arduino15/arduino-cli.yaml" or "~/.arduino15/arduino-cli.yaml")
local sketch = vim.fn.getcwd()

local function term(cmd)
	vim.cmd("botright split | resize 15 | terminal " .. cmd)
end

local compile = string.format("arduino-cli compile --fqbn %s %s", FQBN, sketch)
local upload = string.format("arduino-cli upload -p %s --fqbn %s %s", PORT, FQBN, sketch)
local monitor = string.format("arduino-cli monitor -p %s -c baudrate=%s", PORT, BAUD)

local arduino_keymaps = {
	{
		lhs = "<leader>ac",
		rhs = function()
			term(compile)
		end,
		opts = { desc = "Arduino: compile" },
	},
	{
		lhs = "<leader>au",
		rhs = function()
			term(compile .. " && " .. upload)
		end,
		opts = { desc = "Arduino: compile + upload" },
	},
	{
		lhs = "<leader>am",
		rhs = function()
			term(monitor)
		end,
		opts = { desc = "Arduino: serial monitor" },
	},
	{
		lhs = "<leader>ab",
		rhs = function()
			term("arduino-cli board list")
		end,
		opts = { desc = "Arduino: list boards" },
	},
}

return {
	lsp_servers = { "arduino-language-server" },

	lsp_configs = {
		arduino_language_server = {
			filetypes = { "arduino" },
			root_dir_patterns = { "*.ino" },
			cmd = {
				"arduino-language-server",
				"-cli-config",
				cli_config,
				"-fqbn",
				FQBN,
				"-cli",
				"arduino-cli",
				"-clangd",
				"clangd",
			},
			-- Work around arduino-language-server not supporting semantic tokens properly
			capabilities = {
				textDocument = { semanticTokens = vim.NIL },
				workspace = { semanticTokens = vim.NIL },
			},
		},
	},

	filetype_configs = {
		arduino = {
			opts = { tabstop = 2, shiftwidth = 2, expandtab = true },
			keymaps = arduino_keymaps,
		},
		cpp = {
			opts = { tabstop = 2, shiftwidth = 2, expandtab = true },
			keymaps = arduino_keymaps,
		},
		c = {
			opts = { tabstop = 2, shiftwidth = 2, expandtab = true },
			keymaps = arduino_keymaps,
		},
	},
}
